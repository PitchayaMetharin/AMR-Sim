import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_factory_demo_starts_existing_local_stack_and_supervisors():
    source = (ROOT / "launch" / "factory_demo.launch.py").read_text()
    assert "factory_localization.launch.py" in source
    assert '"headless"' in source
    assert '"initial_x"' in source
    assert '"initial_y"' in source
    assert '"initial_yaw"' in source
    assert '"software_rendering"' in source
    assert '"require_hardware_rendering"' in source
    assert '"factory_attachment"' in source
    assert 'DeclareLaunchArgument("software_rendering", default_value="false")' in source
    assert 'DeclareLaunchArgument("require_hardware_rendering", default_value="true")' in source
    assert 'DeclareLaunchArgument("factory_attachment", default_value="false")' in source
    assert 'executable="manipulation_supervisor_node"' in source
    assert 'executable="factory_supervisor_node"' in source


def test_factory_autonomous_launch_owns_cycle_adapter_and_registry_mapping():
    source = (ROOT / "launch" / "factory_autonomous.launch.py").read_text()
    assert "factory_localization.launch.py" in source
    assert 'executable="cycle_manipulation_supervisor.py"' in source
    assert 'executable="factory_supervisor_node"' in source
    assert 'executable="manipulation_supervisor_node"' not in source
    assert '"product_station_map": product_station_map' in source
    assert '"autonomous_product_ids": autonomous_product_ids' in source
    assert '"dispatch_station_ids": dispatch_station_ids' in source
    assert '"mapping_mode": "false"' in source
    assert 'DeclareLaunchArgument("control_mode", default_value="autonomous")' in source


def test_factory_cli_only_uses_declared_factory_boundaries():
    source = (ROOT / "scripts" / "factory_cli.py").read_text()
    assert "TransportProduct" in source
    assert "SetOperationMode" in source
    assert "FactoryStatus" in source
    assert '"/amr/factory/transport_product"' in source
    assert '"/amr/factory/set_operation_mode"' in source
    assert '"/amr/factory/status"' in source
    assert "wait_for_server(timeout_sec=3.0)" in source
    assert "wait_for_service(timeout_sec=2.0)" in source


def test_factory_cli_transport_default_timeout_preserves_transport_boundary():
    source = (ROOT / "scripts" / "factory_cli.py").read_text()
    transport_parser = source.split('for name in ("send", "enqueue"):', 1)[1]
    transport_parser = transport_parser.split('loop = sub.add_parser', 1)[0]
    assert 'command.add_argument("--timeout", type=float, default=240.0)' in transport_parser
    assert 'client = ActionClient(node, TransportProduct, TRANSPORT_ACTION)' in source


def test_factory_production_exposes_map_yaml_and_mode_without_slam():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    assert 'map_path = os.path.join(factory, "maps", "factory.yaml")' in launch
    assert 'DeclareLaunchArgument("map_yaml", default_value=map_path)' in launch
    assert '"yaml_filename": LaunchConfiguration("map_yaml")' in launch
    assert 'DeclareLaunchArgument("control_mode", default_value="autonomous")' in launch
    assert 'DeclareLaunchArgument("mapping_mode", default_value="false")' in launch
    assert "UnlessCondition(LaunchConfiguration(\"mapping_mode\"))" in launch
    assert "IfCondition(PythonExpression" in launch
    assert 'package="slam_toolbox"' not in launch


def test_factory_mapping_is_online_slam_only_for_global_localization():
    launch = (ROOT / "launch" / "factory_mapping.launch.py").read_text()
    assert 'DeclareLaunchArgument("session_dir")' in launch
    assert 'DeclareLaunchArgument("control_mode", default_value="manual")' in launch
    assert 'DeclareLaunchArgument("headless", default_value="true")' in launch
    assert 'DeclareLaunchArgument("software_rendering", default_value="false")' in launch
    assert 'DeclareLaunchArgument("require_hardware_rendering", default_value="true")' in launch
    assert 'DeclareLaunchArgument("factory_attachment", default_value="true")' in launch
    assert 'DeclareLaunchArgument("initial_x", default_value="-4.5")' in launch
    assert 'DeclareLaunchArgument("initial_y", default_value="0.0")' in launch
    assert 'DeclareLaunchArgument("initial_yaw", default_value="0.0")' in launch
    assert '"factory_attachment": LaunchConfiguration("factory_attachment")' in launch
    assert '"mapping_mode": "true"' in launch
    assert 'package="slam_toolbox"' in launch
    assert 'executable="async_slam_toolbox_node"' in launch
    assert 'package="nav2_map_server"' not in launch
    assert 'package="nav2_amcl"' not in launch
    assert 'package="amr_exploration"' in launch
    assert 'executable="frontier_explorer.py"' in launch
    assert "frontier_explorer.yaml" in launch
    assert 'executable="cycle_manipulation_supervisor.py"' in launch
    assert 'executable="factory_tf_ownership_observer"' in launch
    assert '"allow_product_cycles": False' in launch
    assert 'executable="factory_mapping_readiness.py"' in launch
    assert "_release_after_gate" in launch


def test_factory_mapping_manual_graph_has_no_autonomous_motion_chain():
    launch = (ROOT / "launch" / "factory_mapping.launch.py").read_text()
    manual_graph = launch.split('if control_mode != "autonomous":', 1)[1]
    manual_graph = manual_graph.split('map_ready = _readiness_gate', 1)[0]
    for forbidden in (
            'package="amr_navigation"', 'package="amr_mpc_controller"',
            'package="amr_mission"', 'package="amr_exploration"'):
        assert forbidden not in manual_graph


def test_factory_mapping_autonomous_chain_is_staged_and_fail_closed():
    launch = (ROOT / "launch" / "factory_mapping.launch.py").read_text()
    for stage in ("adapters", "map", "planner", "controller", "mission"):
        assert f'"{stage}"' in launch
    assert launch.index('"adapters"') < launch.index('"map"')
    assert launch.index('"map"') < launch.index('"planner"')
    assert launch.index('"planner"') < launch.index('"controller"')
    assert launch.index('"controller"') < launch.index('"mission"')
    assert "event.returncode == 0" in launch
    assert "Shutdown(" in launch


def test_factory_mapping_uses_the_declared_slam_tf_owner():
    mapper = (ROOT.parent / "amr_slam" / "config" / "mapper.yaml").read_text()
    ownership = json.loads(
        (ROOT.parent / "amr_bringup" / "config" / "interface_ownership.yaml").read_text()
    )
    assert "map_frame: map" in mapper
    assert "odom_frame: odom" in mapper
    assert ownership["runtime_modes"]["mapping"]["/tf:map->odom"] == (
        "slam_toolbox/async_slam_toolbox_node"
    )
    assert ownership["runtime_modes"]["factory"]["/tf:map->odom"] == "nav2_amcl/amcl"


def test_factory_mapping_declares_direct_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    exec_dependencies = {
        dependency.text for dependency in package.findall("exec_depend")
    } | {
        dependency.text for dependency in package.findall("depend")
    }
    assert {
        "amr_exploration", "amr_manipulation", "amr_slam", "slam_toolbox", "std_msgs",
        "tf2_msgs"
    } <= exec_dependencies


def test_mapping_cli_is_installed_and_uses_explicit_artifacts():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    script = (ROOT / "scripts" / "factory_mapping_cli.py").read_text()
    artifacts = (ROOT / "scripts" / "factory_mapping_artifacts.py").read_text()
    acceptance = (ROOT / "scripts" / "factory_mapping_acceptance.py").read_text()
    assert "scripts/factory_mapping_cli.py" in cmake
    assert "scripts/factory_mapping_acceptance.py" in cmake
    assert "scripts/factory_mapping_artifacts.py" in cmake
    assert '"ros2", "run", "nav2_map_server", "map_saver_cli"' in script
    assert '"-t", "/map"' in script
    assert '"map_subscribe_transient_local:=true"' in script
    assert "subprocess.run" in script
    assert "SaveMap" not in script
    assert '"/amr/slam_toolbox/save_map"' not in script
    assert 'SerializePoseGraph, "/amr/slam_toolbox/serialize_map"' in script
    assert "SLAM serialize_map service is unavailable" in script
    assert "mapping_manifest.yaml" in script
    assert "canonical factory maps" in script
    assert "artifact_bundle_sha256" in artifacts
    assert "PROMOTION_ELIGIBLE" in acceptance
    assert "rclpy.spin_once" in acceptance
    assert "publish(" not in acceptance.split("class RosGraphObserver", 1)[1]


def test_tf_ownership_observer_is_built_and_consumed_as_evidence():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {dependency.text for dependency in package.findall("depend")}
    acceptance = (ROOT / "scripts" / "factory_mapping_acceptance.py").read_text()
    source = (ROOT / "src" / "factory_tf_ownership_observer.cpp").read_text()
    assert "factory_tf_ownership_observer" in cmake
    assert "tf2_msgs" in dependencies
    assert "publisher_gid" in source
    assert "endpoint_gid" in source
    assert "TF_OWNERSHIP_TOPIC" in acceptance
    assert "tf_ownership_source" in acceptance
    assert "_parse_tf_ownership_message" in acceptance


def test_manual_factory_control_is_not_started_with_autonomous_controller():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    for include_name in ("include_navigation", "include_mpc", "include_mission"):
        match = re.search(
            rf"(?ms)^    {include_name} = .*?(?=^    (?:include_|product_tag_detector)|\Z)",
            launch,
        )
        assert match is not None
        assert "condition=autonomous_condition" in match.group(0)
