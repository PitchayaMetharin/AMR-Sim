"""Contract tests for the canonical AWS warehouse exploration preset."""

from __future__ import annotations

from collections import Counter
import hashlib
import importlib.util
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = ROOT / "worlds" / "aws_warehouse.sdf"
AWS_LAUNCH_PATH = ROOT / "launch" / "aws_warehouse_exploration.launch.py"
PORTABLE_LAUNCH_PATH = ROOT / "launch" / "portable_exploration.launch.py"
RVIZ_PATH = ROOT / "rviz" / "aws_warehouse_exploration.rviz"
ATTRIBUTION_PATH = ROOT / "assets" / "AWS_WAREHOUSE_FUEL_ATTRIBUTION.md"
EXPECTED_WORLD_SHA256 = "1c1eae5f61e9ec486f9435bdb12c2a6210d50f2da9a001829117bae3bd5623f5"
EXPECTED_SOURCE_SHA256 = "a80de9f6d76e6b589a8b080ccdbce87bd1b88cce9b88cf9de6c81af10602c161"
FUEL_PREFIX = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
MODEL_REVISIONS = {
    "aws_robomaker_warehouse_Bucket_01": 3,
    "aws_robomaker_warehouse_ShelfF_01": 4,
    "aws_robomaker_warehouse_WallB_01": 4,
    "aws_robomaker_warehouse_ShelfE_01": 4,
    "aws_robomaker_warehouse_ShelfD_01": 4,
    "aws_robomaker_warehouse_GroundB_01": 4,
    "aws_robomaker_warehouse_Lamp_01": 4,
    "aws_robomaker_warehouse_ClutteringA_01": 4,
    "aws_robomaker_warehouse_ClutteringC_01": 4,
    "aws_robomaker_warehouse_ClutteringD_01": 4,
    "aws_robomaker_warehouse_TrashCanC_01": 4,
    "aws_robomaker_warehouse_PalletJackB_01": 4,
}
MODEL_COUNTS = {
    "aws_robomaker_warehouse_Bucket_01": 3,
    "aws_robomaker_warehouse_ShelfF_01": 1,
    "aws_robomaker_warehouse_WallB_01": 1,
    "aws_robomaker_warehouse_ShelfE_01": 3,
    "aws_robomaker_warehouse_ShelfD_01": 3,
    "aws_robomaker_warehouse_GroundB_01": 1,
    "aws_robomaker_warehouse_Lamp_01": 1,
    "aws_robomaker_warehouse_ClutteringA_01": 3,
    "aws_robomaker_warehouse_ClutteringC_01": 6,
    "aws_robomaker_warehouse_ClutteringD_01": 1,
    "aws_robomaker_warehouse_TrashCanC_01": 1,
    "aws_robomaker_warehouse_PalletJackB_01": 1,
}
EXPECTED_PLUGINS = [
    "gz-sim-physics-system",
    "gz-sim-user-commands-system",
    "gz-sim-scene-broadcaster-system",
    "gz-sim-contact-system",
    "gz-sim-sensors-system",
    "gz-sim-imu-system",
]


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _substitution_name(value) -> str:
    assert isinstance(value, LaunchConfiguration)
    return value.variable_name[0].text


def _topic_qos(display: dict, field: str = "Topic") -> dict:
    topic = display[field]
    return {
        "value": topic["Value"],
        "reliability": topic["Reliability Policy"],
        "durability": topic["Durability Policy"],
        "history": topic["History Policy"],
        "depth": topic["Depth"],
    }


def test_aws_world_asset_exists_before_any_contract_is_claimed():
    """Fail first when the new production world has not been packaged yet."""

    assert WORLD_PATH.is_file()


def test_aws_world_matches_canonical_transform_and_production_validation():
    portable = _load_module(PORTABLE_LAUNCH_PATH, "portable_launch_for_aws")
    assert WORLD_PATH.stat().st_size == 7954
    assert hashlib.sha256(WORLD_PATH.read_bytes()).hexdigest() == EXPECTED_WORLD_SHA256

    parsed = portable.validate_world(str(WORLD_PATH), "")
    assert parsed.name == "aws_warehouse_world"

    document = ET.parse(WORLD_PATH)
    root = document.getroot()
    assert root.tag == "sdf"
    assert root.attrib == {"version": "1.9"}
    world = root.find("world")
    assert world is not None
    assert [plugin.attrib["filename"] for plugin in world.findall("plugin")] == EXPECTED_PLUGINS
    assert [plugin.attrib["name"] for plugin in world.findall("plugin")] == [
        "gz::sim::systems::Physics",
        "gz::sim::systems::UserCommands",
        "gz::sim::systems::SceneBroadcaster",
        "gz::sim::systems::Contact",
        "gz::sim::systems::Sensors",
        "gz::sim::systems::Imu",
    ]
    assert world.find("./plugin[@filename='gz-sim-sensors-system']/render_engine").text == "ogre2"
    physics = world.findall("physics")
    assert len(physics) == 1
    assert physics[0].attrib["type"] == "ode"
    assert float(physics[0].findtext("max_step_size")) == 0.01

    models = world.findall("model")
    assert len(models) == sum(MODEL_COUNTS.values())
    assert all(model.attrib["name"] != "amr" for model in models)
    assert not any(
        (include.findtext("name") or "").strip() == "amr"
        for include in world.findall(".//include")
    )

    uris = [element.text.strip() for element in root.findall(".//uri")]
    expected_uris = Counter(
        {
            f"{FUEL_PREFIX}{model}/{revision}": count
            for model, revision in MODEL_REVISIONS.items()
            for count in [MODEL_COUNTS[model]]
        }
    )
    assert Counter(uris) == expected_uris
    assert len(uris) == 25
    assert all(uri.startswith(FUEL_PREFIX) for uri in uris)
    assert all(uri.rsplit("/", 1)[-1].isdigit() for uri in uris)
    assert "fuel.ignitionrobotics.org" not in WORLD_PATH.read_text()
    assert "file://" not in WORLD_PATH.read_text()
    assert "/tmp/" not in WORLD_PATH.read_text()


def test_aws_include_is_thin_and_forwards_only_the_portable_public_contract():
    aws_launch = _load_module(AWS_LAUNCH_PATH, "aws_warehouse_launch")
    description = aws_launch.generate_launch_description()
    actions = list(description.entities)
    declarations = [action for action in actions if isinstance(action, DeclareLaunchArgument)]
    assert [action.name for action in declarations] == [
        "headless",
        "rviz",
        "auto_start_exploration",
    ]

    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    include = includes[0]
    source_substitution = include.launch_description_source._LaunchDescriptionSource__location[0]
    source = source_substitution.perform(LaunchContext())
    assert Path(source).resolve() == PORTABLE_LAUNCH_PATH.resolve()

    arguments = dict(include.launch_arguments)
    assert set(arguments) == {
        "world",
        "initial_x",
        "initial_y",
        "initial_z",
        "initial_yaw",
        "resource_paths",
        "headless",
        "rviz",
        "auto_start_exploration",
        "rviz_config",
    }
    assert Path(arguments["world"]).parts[-2:] == ("worlds", WORLD_PATH.name)
    assert arguments["initial_x"] == "0.0"
    assert arguments["initial_y"] == "0.0"
    assert arguments["initial_z"] == "0.12"
    assert arguments["initial_yaw"] == "0.0"
    assert arguments["resource_paths"] == ""
    for name in ("headless", "rviz", "auto_start_exploration"):
        assert _substitution_name(arguments[name]) == name
    assert Path(arguments["rviz_config"]).parts[-2:] == ("rviz", RVIZ_PATH.name)

    # This preset must only declare/include actions: no runtime process is
    # created by the thin wrapper itself.
    assert all(
        isinstance(action, (DeclareLaunchArgument, IncludeLaunchDescription))
        for action in actions
    )


def test_portable_rviz_configuration_has_an_internal_default_seam():
    portable = _load_module(PORTABLE_LAUNCH_PATH, "portable_launch_rviz_seam")
    context = LaunchContext()
    context.launch_configurations.update({
        "headless": "false",
        "rviz": "true",
        "auto_start_exploration": "true",
    })
    validated = portable.validate_world(str(WORLD_PATH), "")
    actions = portable._runtime_actions(context, validated, (0.0, 0.0, 0.12, 0.0))
    rviz_nodes = [
        action
        for action in actions
        if isinstance(action, Node)
        and action.node_package == "rviz2"
        and action.node_executable == "rviz2"
    ]
    # RViz is released with Explorer by the final readiness callback, so it
    # is intentionally nested in that production action graph rather than a
    # top-level action.
    for action in actions:
        if not isinstance(action, RegisterEventHandler):
            continue
        callback = getattr(
            action.event_handler,
            "_OnActionEventBase__on_event",
            None,
        )
        for cell in callback.__closure__ or () if callback else ():
            value = cell.cell_contents
            if isinstance(value, list):
                rviz_nodes.extend(
                    nested
                    for nested in value
                    if isinstance(nested, Node)
                    and nested.node_package == "rviz2"
                    and nested.node_executable == "rviz2"
                )
    assert len(rviz_nodes) == 1
    rviz_argument = rviz_nodes[0]._Node__arguments[1]
    assert isinstance(rviz_argument, LaunchConfiguration)
    assert _substitution_name(rviz_argument) == "rviz_config"
    assert Path(rviz_argument.perform(context)).resolve() == (ROOT / "rviz" / "sensors.rviz").resolve()
    assert 'DeclareLaunchArgument("rviz_config"' not in PORTABLE_LAUNCH_PATH.read_text()


def test_aws_rviz_view_has_required_map_costmap_robot_tf_pose_and_lidar_qos():
    config = yaml.safe_load(RVIZ_PATH.read_text())
    displays = config["Visualization Manager"]["Displays"]
    by_name = {display["Name"]: display for display in displays}
    assert _topic_qos(by_name["SLAM Map"]) == {
        "value": "/map",
        "reliability": "Reliable",
        "durability": "Transient Local",
        "history": "Keep Last",
        "depth": 1,
    }
    for name, topic, updates in (
        (
            "Global Costmap",
            "/amr/global_costmap/costmap",
            "/amr/global_costmap/costmap_updates",
        ),
        (
            "Local Costmap",
            "/amr/local_costmap/costmap",
            "/amr/local_costmap/costmap_updates",
        ),
    ):
        assert _topic_qos(by_name[name]) == {
            "value": topic,
            "reliability": "Reliable",
            "durability": "Transient Local",
            "history": "Keep Last",
            "depth": 1,
        }
        assert _topic_qos(by_name[name], "Update Topic") == {
            "value": updates,
            "reliability": "Reliable",
            "durability": "Volatile",
            "history": "Keep Last",
            "depth": 5,
        }
    assert by_name["AMR Model"]["Class"] == "rviz_default_plugins/RobotModel"
    assert by_name["AMR Model"]["Description Topic"]["Value"] == "/robot_description"
    assert by_name["Sensor Frames"]["Class"] == "rviz_default_plugins/TF"
    assert _topic_qos(by_name["SLAM Pose (mapping localization)"]) == {
        "value": "/amr/pose",
        "reliability": "Reliable",
        "durability": "Volatile",
        "history": "Keep Last",
        "depth": 5,
    }
    for name, topic in (
        ("Front LiDAR Scan (red)", "/amr/sensors/front_lidar/scan"),
        ("Rear LiDAR Scan (green)", "/amr/sensors/rear_lidar/scan"),
    ):
        assert _topic_qos(by_name[name]) == {
            "value": topic,
            "reliability": "Best Effort",
            "durability": "Volatile",
            "history": "Keep Last",
            "depth": 10,
        }
    for display in displays:
        if display["Class"] == "rviz_default_plugins/PointCloud2":
            assert display["Topic"]["Reliability Policy"] == "Best Effort"
            assert display["Topic"]["Durability Policy"] == "Volatile"
    assert config["Visualization Manager"]["Global Options"]["Fixed Frame"] == "map"
    assert "amcl" not in RVIZ_PATH.read_text().lower()
    assert "map_server" not in RVIZ_PATH.read_text().lower()


def test_aws_assets_and_installed_runtime_have_no_preview_or_localization_ownership_leaks():
    source_files = [WORLD_PATH, AWS_LAUNCH_PATH, RVIZ_PATH, ATTRIBUTION_PATH]
    installed_root = ROOT.parent.parent / "install" / "amr_simulation" / "share" / "amr_simulation"
    installed_files = [
        installed_root / "worlds" / WORLD_PATH.name,
        installed_root / "launch" / AWS_LAUNCH_PATH.name,
        installed_root / "rviz" / RVIZ_PATH.name,
        installed_root / "assets" / ATTRIBUTION_PATH.name,
    ]
    for path in source_files + [candidate for candidate in installed_files if candidate.exists()]:
        text = path.read_text().lower()
        assert "amcl" not in text
        assert "map_server" not in text
        assert "/tmp/amr-aws-exploration-preview" not in text
    assert "amcl" not in PORTABLE_LAUNCH_PATH.read_text().lower()
    assert "map_server" not in PORTABLE_LAUNCH_PATH.read_text().lower()


def test_attribution_records_source_fuel_provenance_and_remote_cache_limits():
    text = ATTRIBUTION_PATH.read_text()
    assert "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/industrial-warehouse/4/files/industrial-warehouse.sdf" in text
    assert EXPECTED_SOURCE_SHA256 in text
    assert "OpenRobotics" in text
    assert "Fuel" in text
    for model, revision in MODEL_REVISIONS.items():
        assert f"{FUEL_PREFIX}{model}/{revision}" in text
    lower = text.lower()
    assert "dns/tls" in lower
    assert "cached" in lower and "offline" in lower
    assert "fuel.ignitionrobotics.org" in lower
    assert "not claimed equivalent" in lower
    assert "remote prerequisite" in lower and "not vendored" in lower
    assert "amazon" in lower and "archived" in lower and "aws robomaker" in lower
    assert "per-resource fuel metadata" in lower
    assert "mit-0" in lower and "four vendored factory models" in lower
    assert "12-model bundle" in lower
    assert "no remote model license is asserted" in lower
    assert "/tmp/" not in text


def test_simulation_package_installs_aws_assets_registers_test_and_declares_rviz_plugins():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {
        element.text.strip()
        for element in package.findall("exec_depend")
        if element.text and element.text.strip()
    }
    assert "rviz_default_plugins" in dependencies
    assert "install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})" in cmake
    assert "install(FILES assets/AWS_WAREHOUSE_FUEL_ATTRIBUTION.md" in cmake
    assert "aws_warehouse_exploration_test" in cmake
    assert re.search(r"ament_add_pytest_test\(\s*aws_warehouse_exploration_test\s+test/test_aws_warehouse_exploration\.py\)", cmake)
