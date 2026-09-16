"""World-agnostic, fail-closed simulation launch for autonomous exploration.

The legacy :mod:`amr_simulation.launch` entry point intentionally remains a
small fixed ``amr_world`` smoke launch.  This entry point is different: it
accepts one validated local SDF, derives every world-qualified Gazebo topic
from that SDF, and only then expands the process graph.
"""

from __future__ import annotations

import math
import os
import re
import shlex
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, NamedTuple, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
import xacro


WORLD_NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
FUEL_HOST = "fuel.gazebosim.org"
REQUIRED_PLUGIN_MARKERS = {
    "physics": ("gz-sim-physics-system", "gz::sim::systems::physics"),
    "user_commands": (
        "gz-sim-user-commands-system",
        "gz::sim::systems::usercommands",
    ),
    "scene": (
        "gz-sim-scene-broadcaster-system",
        "gz::sim::systems::scenebroadcaster",
    ),
    "sensors": ("gz-sim-sensors-system", "gz::sim::systems::sensors"),
    "imu": ("gz-sim-imu-system", "gz::sim::systems::imu"),
}


class WorldValidationError(ValueError):
    """Raised before any runtime process is created for an invalid world."""


class ValidatedWorld(NamedTuple):
    """The trusted, parsed world identity used by the launch expansion."""

    path: Path
    name: str
    resource_paths: Tuple[Path, ...] = ()


PORTABLE_READINESS_STAGES = (
    "adapters_authority",
    "slam_map",
    "planner_smoother",
    "controller",
    "mission_final",
)
PORTABLE_INCLUDED_LAUNCH_PACKAGES = (
    "amr_slam",
    "amr_navigation",
    "amr_mpc_controller",
    "amr_control",
    "amr_mission",
)
PORTABLE_ADAPTER_CONFIGURE_DELAY_S = 8.0
PORTABLE_CONTROL_ARGUMENTS = {"require_manipulator_stowed": "true"}
_PORTABLE_STAGE_SUCCESSORS = (
    ("right_gripper_controller", "adapters_authority"),
    ("adapters_authority", "slam_map"),
    ("slam_map", "planner_smoother"),
    ("planner_smoother", "controller"),
    ("controller", "mission_final"),
    ("mission_final", "explorer"),
)


def portable_stage_successors() -> Tuple[Tuple[str, str], ...]:
    """Return the causal one-shot edge labels used by the portable graph."""

    return _PORTABLE_STAGE_SUCCESSORS


def _fail(message: str) -> None:
    raise WorldValidationError(message)


def _finite_float(text: str | None, label: str) -> float:
    if text is None or not text.strip():
        _fail(f"{label} is missing")
    try:
        value = float(text.strip())
    except (TypeError, ValueError) as error:
        _fail(f"{label} is not numeric")
        raise AssertionError from error
    if not math.isfinite(value):
        _fail(f"{label} must be finite")
    return value


def _parse_resource_paths(resource_paths: str | os.PathLike[str] | None) -> Tuple[Path, ...]:
    """Validate the optional, colon-separated local model roots."""

    if resource_paths is None or str(resource_paths).strip() == "":
        return ()
    entries = str(resource_paths).split(os.pathsep)
    roots: List[Path] = []
    for raw_entry in entries:
        entry = raw_entry.strip()
        if not entry:
            _fail("resource_paths contains an empty entry")
        if "://" in entry or entry.startswith("file:"):
            _fail("resource_paths accepts local directories only; URI found")
        path = Path(entry)
        if not path.is_absolute() or not path.is_dir():
            _fail(f"resource path is not an existing absolute directory: {entry}")
        roots.append(path.resolve())
    return tuple(roots)


def _known_package_roots() -> Tuple[Path, ...]:
    """Return package-share roots used for safe local ``model://`` lookup."""

    roots: List[Path] = []
    for package in (
        "amr_description",
        "amr_factory",
        "amr_simulation",
        "kuka_agilus_support",
    ):
        try:
            share = Path(get_package_share_directory(package)).resolve()
        except Exception:
            continue
        roots.extend((share, share / "models", share.parent))
    return tuple(dict.fromkeys(roots))


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _safe_candidate(
    root: Path,
    relative: str,
    *,
    allow_known_symlink: bool = False,
) -> Path | None:
    """Resolve a POSIX URI suffix and reject traversal before existence checks.

    Symlink-install puts package data files in ``install`` as links into the
    package source tree.  For the explicitly trusted package roots, retain the
    logical path when its resolved target preserves the requested relative
    suffix; user/world resource roots remain strict real-path boundaries.
    """

    decoded = urllib.parse.unquote(relative)
    if "\\" in decoded or decoded.startswith("/") or Path(decoded).is_absolute():
        return None
    parts = tuple(part for part in decoded.split("/") if part)
    if any(part in (".", "..") for part in parts):
        return None
    lexical_candidate = root / Path(*parts)
    if not lexical_candidate.exists():
        return None
    candidate = lexical_candidate.resolve()
    if _inside(candidate, root):
        return candidate
    if allow_known_symlink and parts:
        requested_tail = tuple(lexical_candidate.parts[-len(parts):])
        resolved_tail = tuple(candidate.parts[-len(parts):])
        if requested_tail == resolved_tail:
            return lexical_candidate
    return None


def _validate_bare_model_root(
    candidate: Path,
    uri: str,
    *,
    allow_known_symlink: bool = False,
) -> Path:
    """Validate a bare model root through its regular model.config file."""

    if not candidate.is_dir():
        _fail(f"model root is not an existing directory: {uri}")
    config = _safe_candidate(
        candidate, "model.config", allow_known_symlink=allow_known_symlink)
    if config is None or not config.is_file():
        _fail(f"bare model root has no regular model.config: {uri}")
    try:
        config_root = ET.parse(config).getroot()
    except (ET.ParseError, OSError) as error:
        _fail(f"model.config is malformed or unreadable: {uri}: {error}")
    if config_root.tag.rsplit("}", 1)[-1] != "model":
        _fail(f"model.config root is not <model>: {uri}")

    referenced_sdf = []
    for element in config_root.iter():
        if element.tag.rsplit("}", 1)[-1] != "sdf":
            continue
        relative = (element.text or "").strip()
        if not relative or "://" in relative or "\\" in relative:
            _fail(f"model.config contains an unsafe SDF reference: {uri}")
        sdf = _safe_candidate(
            candidate, relative, allow_known_symlink=allow_known_symlink)
        if sdf is None or not sdf.is_file():
            _fail(f"model.config SDF reference is not a safe regular file: {uri}")
        referenced_sdf.append(sdf)
    if not referenced_sdf:
        _fail(f"bare model root has no regular referenced SDF: {uri}")
    return candidate


def _resolve_model_uri(
    uri: str,
    world_dir: Path,
    resource_roots: Sequence[Path],
    known_roots: Sequence[Path],
) -> Path:
    parsed = urllib.parse.urlsplit(uri)
    if parsed.scheme.lower() != "model" or not parsed.netloc:
        _fail(f"malformed model URI: {uri}")
    if parsed.query or parsed.fragment:
        _fail(f"model URI query and fragment are forbidden: {uri}")
    model_name = urllib.parse.unquote(parsed.netloc)
    if not model_name or "/" in model_name or "\\" in model_name:
        _fail(f"malformed model URI model name: {uri}")
    suffix = parsed.path.lstrip("/")
    roots = (world_dir, *resource_roots, *known_roots)
    known_root_set = set(known_roots)
    candidates: List[Path] = []
    for root in roots:
        model_candidate = _safe_candidate(
            root,
            "/".join((model_name, suffix)),
            allow_known_symlink=root in known_root_set,
        )
        if model_candidate is not None:
            candidates.append(model_candidate)
        # A package share itself is also a model root when its basename is the
        # model name (for example model://amr_description/meshes/foo.stl).
        if root.name == model_name:
            direct_candidate = _safe_candidate(
                root, suffix, allow_known_symlink=root in known_root_set)
            if direct_candidate is not None:
                candidates.append(direct_candidate)
    for candidate in candidates:
        if not suffix:
            if candidate.exists():
                candidate_is_known = any(
                    _inside(candidate, root) for root in known_root_set)
                return _validate_bare_model_root(
                    candidate,
                    uri,
                    allow_known_symlink=candidate_is_known,
                )
        elif candidate.is_file():
            return candidate
    _fail(f"unresolved model URI: {uri}")
    raise AssertionError  # _fail always raises


def _validate_fuel_url(uri: str) -> None:
    parsed = urllib.parse.urlsplit(uri)
    if parsed.scheme.lower() not in ("http", "https"):
        _fail(f"unsupported remote resource URI: {uri}")
    if (
        parsed.hostname != FUEL_HOST
        or parsed.username is not None
        or parsed.password is not None
        or parsed.port is not None
        or parsed.query
        or parsed.fragment
    ):
        _fail(f"remote resource is not a bare Fuel model URL: {uri}")
    segments = [segment for segment in parsed.path.split("/") if segment]
    if len(segments) < 5 or "models" not in {segment.lower() for segment in segments[:-1]}:
        _fail(f"remote resource is not a Fuel model URL: {uri}")
    if not segments[-1].isdigit() or int(segments[-1]) <= 0:
        _fail(f"Fuel model URL requires a positive revision: {uri}")


def _validate_xml_uris(
    root: ET.Element,
    world_dir: Path,
    resource_roots: Sequence[Path],
) -> None:
    known_roots = _known_package_roots()
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] != "uri":
            continue
        uri = (element.text or "").strip()
        if not uri:
            _fail("empty resource URI")
        parsed = urllib.parse.urlsplit(uri)
        scheme = parsed.scheme.lower()
        if scheme == "model":
            _resolve_model_uri(uri, world_dir, resource_roots, known_roots)
        elif scheme in ("http", "https"):
            _validate_fuel_url(uri)
        elif scheme == "file" or uri.startswith("file:"):
            _fail(f"file:// resources are forbidden: {uri}")
        elif scheme or "://" in uri:
            _fail(f"unsupported resource URI: {uri}")
        else:
            # Relative SDF resources are allowed only below the world folder.
            candidate = _safe_candidate(world_dir, uri)
            if candidate is None or not candidate.exists():
                _fail(f"unresolved local resource URI: {uri}")


def _validate_poses(root: ET.Element) -> None:
    for pose in root.iter():
        if pose.tag.rsplit("}", 1)[-1] != "pose":
            continue
        tokens = (pose.text or "").split()
        if len(tokens) not in (6, 7):
            _fail("pose must contain six or seven numeric values")
        for index, token in enumerate(tokens):
            _finite_float(token, f"pose[{index}]")


def _validate_world_element(root: ET.Element) -> Tuple[ET.Element, str]:
    if root.tag.rsplit("}", 1)[-1] != "sdf" or root.attrib.get("version") != "1.9":
        _fail("world root must be SDF version exactly 1.9")
    worlds = [child for child in list(root) if child.tag.rsplit("}", 1)[-1] == "world"]
    if len(worlds) != 1:
        _fail("SDF must contain exactly one direct world")
    world = worlds[0]
    if any(
        element is not world and element.tag.rsplit("}", 1)[-1] == "world"
        for element in world.iter()
    ):
        _fail("nested world elements are not permitted")
    name = (world.attrib.get("name") or "").strip()
    if not name or WORLD_NAME_RE.fullmatch(name) is None:
        _fail("world name must be non-empty and topic-safe")
    return world, name


def _validate_plugins(world: ET.Element) -> None:
    plugins = []
    for plugin in world.findall("plugin"):
        # filename is the loadable system contract.  A descriptive plugin
        # name alone must not make a missing/broken shared object pass.
        filename = Path((plugin.attrib.get("filename") or "").lower()).name.replace(" ", "")
        haystack = filename
        plugins.append(haystack)
    for requirement, markers in REQUIRED_PLUGIN_MARKERS.items():
        if not any(
            any(
                plugin == marker
                or plugin.startswith(f"{marker}.")
                or plugin.startswith(f"lib{marker}.")
                for marker in markers
            )
            for plugin in plugins
        ):
            _fail(f"required Gazebo {requirement} system plugin is missing")


def _validate_physics(world: ET.Element) -> None:
    physics_elements = world.findall("physics")
    if not physics_elements:
        _fail("world physics is missing")
    found_step = False
    for physics in physics_elements:
        max_step = physics.find("max_step_size")
        if max_step is None:
            _fail("physics max_step_size is missing")
        found_step = True
        value = _finite_float(max_step.text, "physics max_step_size")
        if value <= 0.0 or value > 0.01:
            _fail("physics max_step_size must be positive and no greater than 0.01")
        for tag in ("real_time_factor", "real_time_update_rate"):
            for element in physics.findall(f".//{tag}"):
                value = _finite_float(element.text, f"physics {tag}")
                if value <= 0.0:
                    _fail(f"physics {tag} must be positive")
    if not found_step:
        _fail("physics max_step_size is missing")


def _reject_preexisting_amr(world: ET.Element) -> None:
    for model in world.iter("model"):
        if (model.attrib.get("name") or "").strip() == "amr":
            _fail("world already contains the reserved amr model")
    for include in world.iter("include"):
        if (include.findtext("name") or "").strip() == "amr":
            _fail("world include already resolves to the reserved amr model")
        uri = (include.findtext("uri") or "").strip()
        parsed = urllib.parse.urlsplit(uri)
        if parsed.scheme.lower() == "model" and urllib.parse.unquote(parsed.netloc) == "amr":
            _fail("world include URI resolves to the reserved amr model")


def validate_world(
    world: str | os.PathLike[str],
    resource_paths: str | os.PathLike[str] | None = "",
) -> ValidatedWorld:
    """Validate a local SDF and all of its directly declared resources."""

    world_arg = str(world)
    if "://" in world_arg or urllib.parse.urlsplit(world_arg).scheme:
        _fail("world must be a local path; direct URI worlds are forbidden")
    path = Path(world_arg)
    if not path.is_absolute() or path.suffix.lower() != ".sdf":
        _fail("world must be an absolute local .sdf path")
    if not path.is_file():
        _fail("world must be an existing regular local SDF file")
    path = path.resolve()
    roots = _parse_resource_paths(resource_paths)
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as error:
        _fail(f"world XML is malformed or unreadable: {error}")
    world_element, name = _validate_world_element(root)
    _validate_plugins(world_element)
    _validate_physics(world_element)
    _validate_poses(world_element)
    _reject_preexisting_amr(world_element)
    _validate_xml_uris(root, path.parent, roots)
    return ValidatedWorld(path=path, name=name, resource_paths=roots)


def validate_spawn_pose(values: Iterable[str | float]) -> Tuple[float, float, float, float]:
    values = tuple(values)
    if len(values) != 4:
        raise WorldValidationError("spawn pose requires x, y, z, and yaw")
    parsed = tuple(_finite_float(str(value), f"spawn pose[{index}]") for index, value in enumerate(values))
    return parsed  # type: ignore[return-value]


def bridge_arguments(world_name: str) -> List[str]:
    """Return the stable bridge contract with only the world edge dynamic."""

    if not world_name or WORLD_NAME_RE.fullmatch(world_name) is None:
        raise WorldValidationError("bridge world name is not topic-safe")
    return [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/amr/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist",
        "/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/model/amr/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        f"/world/{world_name}/model/amr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/amr/simulation/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/amr/simulation/sensors/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/amr/simulation/sensors/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/amr/simulation/sensors/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/amr/simulation/sensors/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/amr/simulation/sensors/product_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/amr/simulation/sensors/product_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/amr/simulation/sensors/product_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    ]


def _managed_node(
    package: str,
    executable: str,
    *,
    name: str | None = None,
    parameters: Sequence[object] = (),
) -> Tuple[LifecycleNode, List[object]]:
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=name or executable,
        namespace="/amr",
        output="screen",
        parameters=[{"use_sim_time": True}, *parameters],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )
    # Register every transition handler before a configure event is emitted.
    return node, [activate, configure]


def _shutdown_on_exit(action: object, label: str) -> RegisterEventHandler:
    def on_exit(event, _context):
        # A required long-running process exiting cleanly is still a graph
        # failure: keeping the remaining graph alive would be unsafe.
        return [Shutdown(reason=f"required portable process exited: {label}")]

    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def _release_one_shot(next_actions: Sequence[object], label: str):
    def on_exit(event, _context):
        if getattr(event, "returncode", None) == 0:
            return list(next_actions)
        return [Shutdown(reason=f"portable one-shot failed: {label}")]

    return on_exit


def _readiness_node(stage: str) -> Node:
    if stage not in PORTABLE_READINESS_STAGES:
        raise ValueError(f"unknown portable readiness stage: {stage}")
    return Node(
        package="amr_simulation",
        executable="portable_exploration_readiness.py",
        name=f"portable_exploration_readiness_{stage}",
        namespace="/amr",
        parameters=[{"use_sim_time": True, "stage": stage}],
        output="screen",
    )


def _package_launch_include(
    package: str,
    *,
    arguments: dict[str, str] | None = None,
) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / "launch" / f"{package}.launch.py"
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def _shutdown_on_required_process_exit(expected_one_shots: set[object]) -> RegisterEventHandler:
    """Fail closed on every unexpected process exit, including return code 0."""

    def on_exit(event, context):
        if getattr(context, "is_shutdown", False):
            return []
        if getattr(event, "action", None) in expected_one_shots:
            return []
        action = getattr(event, "action", None)
        label = getattr(action, "executable", None) or getattr(action, "name", None) or "unknown"
        return [Shutdown(reason=f"required portable process exited: {label}")]

    return RegisterEventHandler(OnProcessExit(on_exit=on_exit))


def _runtime_actions(context, validated: ValidatedWorld, pose: Tuple[float, float, float, float]):
    """Build the portable graph as causal one-shot and readiness stages."""

    simulation = Path(get_package_share_directory("amr_simulation"))
    description = Path(get_package_share_directory("amr_description"))
    controller_config = description / "config" / "phase14_mobile_manipulator_controllers.yaml"
    robot_xacro = description / "urdf" / "phase14_mobile_manipulator.urdf.xacro"
    joint_state_topic = f"/world/{validated.name}/model/amr/joint_state"
    robot_xml = xacro.process_file(
        str(robot_xacro),
        mappings={
            "include_generic_payload": "false",
            "loaded_product": "false",
            "factory_attachment": "false",
            "controller_config": str(controller_config),
            "joint_state_topic": joint_state_topic,
        },
    ).toxml()
    robot_description = {"robot_description": robot_xml, "use_sim_time": True}

    resource_roots = [validated.path.parent, *validated.resource_paths]
    for root in _known_package_roots():
        if root.is_dir():
            resource_roots.append(root)
    resource_value = os.pathsep.join(str(path) for path in dict.fromkeys(resource_roots))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gz_args = "-r " + ("-s " if headless else "") + f"-v 2 {shlex.quote(str(validated.path))}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_arguments(validated.name),
        remappings=[
            ("/model/amr/cmd_vel", "/amr/simulation/base/cmd_vel"),
            ("/model/amr/odometry", "/amr/simulation/base/odometry"),
            ("/model/amr/pose", "/amr/simulation/ground_truth/pose"),
            (joint_state_topic, "/amr/simulation/base/joint_states"),
            ("/amr/simulation/sensors/front_lidar/scan/points", "/amr/simulation/sensors/front_lidar/points"),
            ("/amr/simulation/sensors/rear_lidar/scan/points", "/amr/simulation/sensors/rear_lidar/points"),
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", validated.name,
            "-name", "amr",
            "-param", "robot_description",
            "-x", str(pose[0]),
            "-y", str(pose[1]),
            "-z", str(pose[2]),
            "-Y", str(pose[3]),
        ],
        parameters=[robot_description],
        output="screen",
    )
    joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )
    left_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )
    right_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_right_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    # Register every adapter handler before the delayed base configure event.
    base, (base_activate, base_configure) = _managed_node(
        "amr_base_adapter", "base_adapter_node")
    front, (front_activate, front_configure) = _managed_node(
        "amr_sensor_adapters", "front_lidar_adapter_node")
    rear, (rear_activate, rear_configure) = _managed_node(
        "amr_sensor_adapters", "rear_lidar_adapter_node")
    imu, (imu_activate, imu_configure) = _managed_node(
        "amr_sensor_adapters", "imu_adapter_node")
    product_camera, (product_activate, product_configure) = _managed_node(
        "amr_sensor_adapters", "product_camera_adapter_node")
    adapter_initial: List[object] = [
        base, base_activate,
        front, front_activate,
        rear, rear_activate,
        imu, imu_activate,
        product_camera, product_activate,
        TimerAction(period=PORTABLE_ADAPTER_CONFIGURE_DELAY_S, actions=[base_configure]),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=base,
            goal_state="active",
            entities=[front_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=front,
            goal_state="active",
            entities=[rear_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=rear,
            goal_state="active",
            entities=[imu_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=imu,
            goal_state="active",
            entities=[product_configure],
        )),
    ]

    wheel, (wheel_activate, wheel_configure) = _managed_node(
        "amr_localization",
        "wheel_odometry_node",
        parameters=[{"wheel_radius": 0.1128, "wheel_separation": 0.566}],
    )
    front_perception, (front_perception_activate, front_perception_configure) = _managed_node(
        "amr_perception", "front_lidar_perception_node")
    rear_perception, (rear_perception_activate, rear_perception_configure) = _managed_node(
        "amr_perception", "rear_lidar_perception_node")
    health, (health_activate, health_configure) = _managed_node(
        "amr_health", "health_supervisor_node")
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace="/amr",
        parameters=[str(Path(get_package_share_directory("amr_localization")) / "config" / "ekf.yaml")],
        remappings=[("odometry/filtered", "/amr/localization/odometry")],
        output="screen",
    )
    control_include = _package_launch_include(
        "amr_control", arguments=PORTABLE_CONTROL_ARGUMENTS)
    stow = Node(
        package="amr_simulation",
        executable="portable_stow_authority.py",
        name="portable_stow_authority",
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    readiness_nodes = {
        stage: _readiness_node(stage) for stage in PORTABLE_READINESS_STAGES
    }
    readiness_initial = readiness_nodes[PORTABLE_READINESS_STAGES[0]]

    # Only the adapter/authority group is released by the right gripper.
    # The product-camera active transition then releases the remaining
    # adapter, perception, control, health, stow, and first readiness actions.
    adapter_tail: List[object] = [
        wheel, wheel_activate,
        front_perception, front_perception_activate,
        rear_perception, rear_perception_activate,
        health, health_activate,
        wheel_configure, front_perception_configure,
        rear_perception_configure, health_configure,
        ekf,
        control_include,
        stow,
        readiness_initial,
    ]
    adapter_initial.append(RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=product_camera,
        goal_state="active",
        entities=adapter_tail,
    )))

    # Keep the existing package barriers/timers for each later causal stage.
    slam_include = _package_launch_include("amr_slam")
    navigation_include = _package_launch_include("amr_navigation")
    mpc_include = _package_launch_include("amr_mpc_controller")
    mission_include = _package_launch_include("amr_mission")

    explorer_config = str(Path(get_package_share_directory("amr_exploration")) / "config" / "frontier_explorer.yaml")
    explorer = Node(
        package="amr_exploration",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        namespace="/amr",
        parameters=[
            explorer_config,
            {"autostart": ParameterValue(LaunchConfiguration("auto_start_exploration"), value_type=bool)},
        ],
        output="screen",
    )
    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=str(simulation / "rviz" / "sensors.rviz"),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="portable_exploration_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    readiness_stage_actions = {
        "adapters_authority": [slam_include, readiness_nodes["slam_map"]],
        "slam_map": [navigation_include, readiness_nodes["planner_smoother"]],
        "planner_smoother": [mpc_include, readiness_nodes["controller"]],
        "controller": [mission_include, readiness_nodes["mission_final"]],
        "mission_final": [explorer, rviz],
    }
    readiness_release_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=readiness_nodes[stage],
            on_exit=_release_one_shot(
                readiness_stage_actions[stage], f"{stage} readiness"),
        ))
        for stage in PORTABLE_READINESS_STAGES
    ]

    expected_one_shots = {
        spawn,
        joint_broadcaster,
        arm_controller,
        left_gripper,
        right_gripper,
        *readiness_nodes.values(),
    }
    return [
        _shutdown_on_required_process_exit(expected_one_shots),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_value),
        gazebo,
        state_publisher,
        bridge,
        *readiness_release_handlers,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=_release_one_shot([joint_broadcaster], "robot insertion"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_broadcaster,
            on_exit=_release_one_shot([arm_controller], "joint state broadcaster"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller,
            on_exit=_release_one_shot([left_gripper], "arm controller"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=left_gripper,
            on_exit=_release_one_shot([right_gripper], "left gripper controller"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=right_gripper,
            on_exit=_release_one_shot(adapter_initial, "right gripper controller"),
        )),
        spawn,
    ]


def _expand_runtime(context):
    world_arg = LaunchConfiguration("world").perform(context)
    resources = LaunchConfiguration("resource_paths").perform(context)
    validated = validate_world(world_arg, resources)
    pose = validate_spawn_pose(
        (
            LaunchConfiguration("initial_x").perform(context),
            LaunchConfiguration("initial_y").perform(context),
            LaunchConfiguration("initial_z").perform(context),
            LaunchConfiguration("initial_yaw").perform(context),
        )
    )
    return _runtime_actions(context, validated, pose)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world", description="Required absolute local SDF 1.9 world path (never a URI)."),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.12"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("resource_paths", default_value=""),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("auto_start_exploration", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_expand_runtime),
    ])
