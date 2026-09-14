"""Launch the factory world in bounded online-mapping mode.

The shared factory launch still owns Gazebo, the robot, adapters, local EKF,
and command arbitration.  This entry point owns the mapping-specific graph:
SLAM and the cycle adapter are available in both modes, while autonomous
mapping releases planning, control, mission, and exploration one observable
readiness boundary at a time.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_mapping_session(context):
    """Require a narrow, explicit session path before starting the graph."""
    raw_path = LaunchConfiguration("session_dir").perform(context).strip()
    if not raw_path:
        raise RuntimeError(
            "session_dir is required for factory mapping; choose a run-specific directory"
        )
    session_dir = Path(raw_path).expanduser().resolve()
    if session_dir == Path("/"):
        raise RuntimeError("session_dir must not be the filesystem root")
    canonical_maps = {
        Path(get_package_share_directory("amr_factory"), "maps").resolve(),
        Path(__file__).resolve().parents[1] / "maps",
    }
    if any(session_dir == path or path in session_dir.parents for path in canonical_maps):
        raise RuntimeError("session_dir must not be the canonical factory maps directory")


def validate_mapping_options(context):
    """Reject unsupported commissioning options before starting processes."""
    control_mode = LaunchConfiguration("control_mode").perform(context).strip().lower()
    if control_mode not in {"manual", "autonomous"}:
        raise RuntimeError("control_mode must be manual or autonomous")
    factory_attachment = (
        LaunchConfiguration("factory_attachment").perform(context).strip().lower())
    if factory_attachment != "true":
        raise RuntimeError(
            "factory mapping requires factory_attachment=true for bounded cycle status proof")


def _load_autonomous_mapping(factory):
    """Derive the mapping adapter inputs from the factory registries."""
    config_dir = os.path.join(factory, "config")
    with open(os.path.join(config_dir, "products.yaml"), encoding="utf-8") as stream:
        products = yaml.safe_load(stream) or {}
    with open(os.path.join(config_dir, "stations.yaml"), encoding="utf-8") as stream:
        stations = yaml.safe_load(stream) or {}

    raw_products = products.get("products")
    raw_stations = stations.get("stations")
    if not isinstance(raw_products, dict) or not isinstance(raw_stations, dict):
        raise RuntimeError("factory mapping registries must contain mappings")

    autonomous = []
    for model, entry in raw_products.items():
        if not isinstance(model, str) or not isinstance(entry, dict):
            raise RuntimeError("product registry contains an invalid entry")
        tag_id = entry.get("tag_id")
        if tag_id == 103 and entry.get("autonomous_enabled") is True:
            raise RuntimeError("Product103 must remain disabled for mapping cycles")
        if entry.get("autonomous_enabled") is not True:
            continue
        station_id = entry.get("pickup_station")
        if (not isinstance(station_id, str) or not station_id or
                isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id <= 0 or
                tag_id == 103):
            raise RuntimeError(f"autonomous product {model} has an invalid mapping")
        station = raw_stations.get(station_id)
        if not isinstance(station, dict) or station.get("role") != "pickup":
            raise RuntimeError(
                f"autonomous product {model} references a non-pickup station")
        autonomous.append((station_id, tag_id))

    dispatch_stations = sorted(
        station_id for station_id, entry in raw_stations.items()
        if isinstance(station_id, str) and isinstance(entry, dict) and
        entry.get("role") == "dispatch")
    if not autonomous or not dispatch_stations:
        raise RuntimeError("factory mapping registries have no usable route")
    autonomous.sort()
    return (
        [f"{station_id}={product_id}" for station_id, product_id in autonomous],
        [product_id for _, product_id in autonomous],
        dispatch_stations,
    )


def _readiness_gate(stage, name):
    """Create one bounded observable gate process for the launch graph."""
    return Node(
        package="amr_factory",
        executable="factory_mapping_readiness.py",
        name=name,
        namespace="/amr",
        parameters=[{
            "stage": stage,
            "timeout_sec": 60.0,
        }],
        output="screen",
    )


def _release_after_gate(stage, entities):
    """Release downstream actions only after a zero exit code."""
    def callback(event, _context):
        if event.returncode == 0:
            return entities
        return Shutdown(
            reason=f"factory mapping {stage} readiness failed with exit code "
                   f"{event.returncode}")
    return callback


def _shutdown_on_nonzero(process_name):
    """Fail closed if a required long-running process exits unexpectedly."""
    def callback(event, _context):
        if event.returncode not in (None, 0):
            return Shutdown(
                reason=f"factory mapping {process_name} exited with code "
                       f"{event.returncode}")
        return []
    return callback


def _build_mapping_graph(context):
    """Build the mode-specific graph after launch arguments are resolved."""
    factory = get_package_share_directory("amr_factory")
    control_mode = LaunchConfiguration("control_mode").perform(context).strip().lower()
    product_station_map, autonomous_product_ids, dispatch_station_ids = (
        _load_autonomous_mapping(factory)
    )

    # The shared launch must not create its AMCL-gated autonomous actions in
    # mapping mode: SLAM owns map->odom and this file owns their staged release.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(factory, "launch", "factory_localization.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "software_rendering": LaunchConfiguration("software_rendering"),
            "require_hardware_rendering": LaunchConfiguration(
                "require_hardware_rendering"),
            "factory_attachment": LaunchConfiguration("factory_attachment"),
            "control_mode": "manual",
            "mapping_mode": "true",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )
    slam_parameters = os.path.join(
        get_package_share_directory("amr_slam"), "config", "mapper.yaml")
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="/amr",
        parameters=[slam_parameters],
        output="screen",
    )
    manipulation = Node(
        package="amr_manipulation",
        executable="cycle_manipulation_supervisor.py",
        name="manipulation_supervisor_node",
        namespace="/amr",
        parameters=[{
            "use_sim_time": True,
            "allow_product_cycles": False,
            "product_station_map": product_station_map,
            "autonomous_product_ids": autonomous_product_ids,
            "dispatch_station_ids": dispatch_station_ids,
            "internal_status_topic": "/amr/manipulation/internal/status",
            "internal_cancel_service": "/amr/manipulation/internal/cancel_cycle_motion",
            "bootstrap_service": "/amr/simulation/attachment_bootstrap/verify",
        }],
        output="screen",
    )
    tf_ownership_observer = Node(
        package="amr_factory",
        executable="factory_tf_ownership_observer",
        name="tf_ownership_observer",
        namespace="/amr",
        output="screen",
    )

    adapters_ready = _readiness_gate(
        "adapters", "factory_mapping_adapters_readiness")
    actions = [localization, manipulation, tf_ownership_observer, adapters_ready]
    if control_mode != "autonomous":
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=adapters_ready,
            on_exit=_release_after_gate("adapters", [slam]),
        )))
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=slam,
            on_exit=_shutdown_on_nonzero("slam_toolbox"),
        )))
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=tf_ownership_observer,
            on_exit=_shutdown_on_nonzero("tf_ownership_observer"),
        )))
        return actions

    map_ready = _readiness_gate("map", "factory_mapping_map_readiness")
    planner_ready = _readiness_gate(
        "planner", "factory_mapping_planner_readiness")
    controller_ready = _readiness_gate(
        "controller", "factory_mapping_controller_readiness")
    mission_ready = _readiness_gate(
        "mission", "factory_mapping_mission_readiness")
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_navigation"),
            "launch", "amr_navigation.launch.py")))
    mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_mpc_controller"),
            "launch", "amr_mpc_controller.launch.py")))
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_mission"),
            "launch", "amr_mission.launch.py")))
    explorer = Node(
        package="amr_exploration",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        namespace="/amr",
        parameters=[os.path.join(
            get_package_share_directory("amr_exploration"),
            "config", "frontier_explorer.yaml")],
        output="screen",
    )

    # Each release callback checks the gate's exit code.  A failed or timed
    # out observation therefore shuts the graph down before any dependent
    # process is created.
    actions.extend([
        RegisterEventHandler(OnProcessExit(
            target_action=adapters_ready,
            on_exit=_release_after_gate("adapters", [slam, map_ready]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=map_ready,
            on_exit=_release_after_gate("map", [navigation, planner_ready]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=planner_ready,
            on_exit=_release_after_gate("planner+smoother", [mpc, controller_ready]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controller_ready,
            on_exit=_release_after_gate("controller", [mission, mission_ready]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=mission_ready,
            on_exit=_release_after_gate("mission", [explorer]),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=slam,
            on_exit=_shutdown_on_nonzero("slam_toolbox"),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=tf_ownership_observer,
            on_exit=_shutdown_on_nonzero("tf_ownership_observer"),
        )),
    ])
    return actions


def generate_launch_description():
    factory = get_package_share_directory("amr_factory")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("require_hardware_rendering", default_value="true"),
        DeclareLaunchArgument("factory_attachment", default_value="true"),
        DeclareLaunchArgument("control_mode", default_value="manual"),
        DeclareLaunchArgument("session_dir"),
        # Kept explicit for the shared launch contract; mapping mode never
        # starts map_server and therefore never reads this file.
        DeclareLaunchArgument(
            "map_yaml", default_value=os.path.join(factory, "maps", "factory.yaml")),
        DeclareLaunchArgument("initial_x", default_value="-4.5"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=validate_mapping_options),
        OpaqueFunction(function=validate_mapping_session),
        SetEnvironmentVariable(
            name="AMR_FACTORY_MAPPING_SESSION_DIR",
            value=LaunchConfiguration("session_dir")),
        OpaqueFunction(function=_build_mapping_graph),
    ])
