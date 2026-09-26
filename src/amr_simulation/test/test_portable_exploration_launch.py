"""Failing-first contract tests for the portable exploration launch."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import stat
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from launch import LaunchContext
import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT / "launch" / "portable_exploration.launch.py"


def _load_launch():
    spec = importlib.util.spec_from_file_location("portable_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_existing_amr_world_validates_and_dynamic_bridge_names_are_derived():
    launch = _load_launch()
    world = ROOT / "worlds" / "amr_world.sdf"
    parsed = launch.validate_world(str(world), "")

    assert parsed.name == "amr_world"
    bridge = launch.bridge_arguments(parsed.name)
    assert any(f"/world/{parsed.name}/model/amr/joint_state" in item for item in bridge)
    assert any("/model/amr/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist" in item for item in bridge)
    assert any("/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" in item for item in bridge)
    assert any("/model/amr/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose" in item for item in bridge)
    assert any("/amr/simulation/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU" in item for item in bridge)
    assert any("/amr/simulation/sensors/front_lidar/scan/points@sensor_msgs/msg/PointCloud2" in item for item in bridge)
    assert any("/amr/simulation/sensors/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" in item for item in bridge)
    assert any("/amr/simulation/sensors/product_camera/image" in item for item in bridge)
    assert any("/amr/simulation/sensors/product_camera/depth_image" in item for item in bridge)
    assert any("/amr/simulation/sensors/product_camera/camera_info" in item for item in bridge)


def test_simulation_diagnostics_bridge_and_generated_robot_plugin_are_opt_in():
    launch = _load_launch()

    default_bridge = launch.bridge_arguments("amr_world")
    diagnostic_bridge = launch.bridge_arguments(
        "amr_world", simulation_diagnostics=True)

    assert not any("/amr/simulation/diagnostics/" in item for item in default_bridge)
    assert (
        "/amr/simulation/diagnostics/contacts"
        "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
    ) in diagnostic_bridge
    assert (
        "/amr/simulation/diagnostics/delivered_cmd_vel"
        "@geometry_msgs/msg/TwistStamped[gz.msgs.Twist"
    ) in diagnostic_bridge
    assert (
        "/amr/simulation/diagnostics/contact_coverage"
        "@std_msgs/msg/String[gz.msgs.StringMsg"
    ) in diagnostic_bridge

    original = "<robot name='amr'><link name='base_link'/></robot>"
    unchanged = launch.robot_xml_with_simulation_diagnostics(
        original, enabled=False)
    instrumented = launch.robot_xml_with_simulation_diagnostics(
        original, enabled=True)
    assert unchanged == original
    original_root = ET.fromstring(original)
    instrumented_root = ET.fromstring(instrumented)
    assert original_root.find("gazebo") is None
    plugin = instrumented_root.find("./gazebo/plugin")
    assert plugin is not None
    assert plugin.attrib == {
        "filename": "amr-exploration-evidence-system",
        "name": "amr_simulation::ExplorationEvidenceSystem",
    }
    assert plugin.findtext("command_topic") == "/model/amr/cmd_vel"
    assert plugin.findtext("contacts_topic") == "/amr/simulation/diagnostics/contacts"
    assert (
        plugin.findtext("delivered_command_topic")
        == "/amr/simulation/diagnostics/delivered_cmd_vel"
    )
    assert (
        plugin.findtext("coverage_topic")
        == "/amr/simulation/diagnostics/contact_coverage"
    )


@pytest.mark.parametrize(
    "mutator",
    [
        lambda text: text.replace('version="1.9"', 'version="1.8"', 1),
        lambda text: text.replace('<world name="amr_world">', '<world name="">', 1),
        lambda text: text.replace('</world>', '<world name="second"/></world>', 1),
        lambda text: text.replace('gz-sim-imu-system', 'missing-imu-system', 1),
        lambda text: text.replace('<max_step_size>0.001</max_step_size>', '<max_step_size>nan</max_step_size>', 1),
        lambda text: text.replace('<model name="ground">', '<model name="amr">', 1),
        lambda text: text.replace('<pose>3 0 1 0 0 0</pose>', '<pose>inf 0 1 0 0 0</pose>', 1),
    ],
)
def test_invalid_worlds_fail_closed(tmp_path, mutator):
    launch = _load_launch()
    source = (ROOT / "worlds" / "amr_world.sdf").read_text()
    path = tmp_path / "invalid.sdf"
    path.write_text(mutator(source))

    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(path), "")


@pytest.mark.parametrize(
    "world_arg",
    ["relative.sdf", "file:///tmp/world.sdf", "https://example.invalid/world.sdf"],
)
def test_direct_world_uri_or_relative_path_is_rejected(world_arg):
    launch = _load_launch()
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(world_arg, "")


def test_resource_paths_and_uri_resolution_are_fail_closed(tmp_path):
    launch = _load_launch()
    world_dir = tmp_path / "world"
    world_dir.mkdir()
    model_dir = world_dir / "crate"
    model_dir.mkdir()
    (model_dir / "model.sdf").write_text("<sdf version='1.9'/>")
    world = world_dir / "world.sdf"
    source = (ROOT / "worlds" / "amr_world.sdf").read_text()
    world.write_text(source.replace("</world>", "<include><uri>model://crate/model.sdf</uri></include></world>"))
    assert launch.validate_world(str(world), "").name == "amr_world"

    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "bad.sdf").write_text("<sdf version='1.9'/>")
    world.write_text(source.replace("</world>", "<include><uri>model://crate/../outside/bad.sdf</uri></include></world>"))
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(world), "")

    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(ROOT / "worlds" / "amr_world.sdf"), "file:///tmp")

    world.write_text(source.replace("</world>", "<include><uri>/etc/passwd</uri></include></world>"))
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(world), "")


def test_bare_model_root_requires_safe_model_config_and_regular_sdf(tmp_path):
    launch = _load_launch()
    source = (ROOT / "worlds" / "amr_world.sdf").read_text()

    def make_world(uri: str, name: str) -> Path:
        path = tmp_path / f"{name}.sdf"
        include = f"<include><uri>{uri}</uri></include>"
        path.write_text(source.replace("</world>", include + "</world>"))
        return path

    model_root = tmp_path / "models" / "crate"
    model_root.mkdir(parents=True)
    world = make_world("model://crate", "empty_config")
    (model_root / "model.config").write_text("")
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(world), str(tmp_path / "models"))

    (model_root / "model.config").write_text(
        "<model><name>crate</name><sdf>../outside.sdf</sdf></model>"
    )
    (tmp_path / "outside.sdf").write_text("<sdf version='1.9'/>")
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(world), str(tmp_path / "models"))

    (model_root / "model.config").write_text(
        "<model><name>crate</name><sdf>missing.sdf</sdf></model>"
    )
    with pytest.raises(launch.WorldValidationError):
        launch.validate_world(str(world), str(tmp_path / "models"))

    (model_root / "model.config").write_text(
        "<model><name>crate</name><sdf version='1.9'>model.sdf</sdf></model>"
    )
    (model_root / "model.sdf").write_text("<sdf version='1.9'/>")
    assert launch.validate_world(str(world), str(tmp_path / "models")).name == "amr_world"


def test_explicit_model_resource_file_does_not_require_model_config(tmp_path):
    launch = _load_launch()
    source = (ROOT / "worlds" / "amr_world.sdf").read_text()
    model_root = tmp_path / "models" / "explicit"
    model_root.mkdir(parents=True)
    (model_root / "model.sdf").write_text("<sdf version='1.9'/>")
    world = tmp_path / "explicit_world.sdf"
    world.write_text(source.replace(
        "</world>",
        "<include><uri>model://explicit/model.sdf</uri></include></world>",
    ))
    assert launch.validate_world(str(world), str(tmp_path / "models")).name == "amr_world"


def test_fuel_model_urls_require_positive_revision_without_query_or_fragment(tmp_path):
    launch = _load_launch()
    source = (ROOT / "worlds" / "amr_world.sdf").read_text()
    world = tmp_path / "world.sdf"
    prefix = "<include><uri>{}</uri></include>"
    world.write_text(source.replace("</world>", prefix.format("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bucket/3") + "</world>"))
    launch.validate_world(str(world), "")
    for url in (
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bucket/0",
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bucket",
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bucket/3?x=1",
        "https://example.invalid/1.0/OpenRobotics/models/Bucket/3",
    ):
        world.write_text(source.replace("</world>", prefix.format(url) + "</world>"))
        with pytest.raises(launch.WorldValidationError):
            launch.validate_world(str(world), "")


def test_launch_contract_is_isolated_and_stages_explorer_after_readiness():
    text = LAUNCH_PATH.read_text()
    assert 'DeclareLaunchArgument("world"' in text
    assert 'DeclareLaunchArgument("initial_x", default_value="0.0"' in text
    assert 'DeclareLaunchArgument("initial_y", default_value="0.0"' in text
    assert 'DeclareLaunchArgument("initial_yaw", default_value="0.0"' in text
    assert 'DeclareLaunchArgument("initial_z", default_value="0.12"' in text
    assert 'DeclareLaunchArgument("resource_paths", default_value=""' in text
    assert 'DeclareLaunchArgument("headless", default_value="false"' in text
    assert 'DeclareLaunchArgument("software_rendering", default_value="auto"' in text
    assert 'DeclareLaunchArgument("rviz", default_value="true"' in text
    assert 'DeclareLaunchArgument("auto_start_exploration", default_value="true"' in text
    assert 'DeclareLaunchArgument("simulation_diagnostics", default_value="false"' in text
    assert 'DeclareLaunchArgument("localization_mode", default_value="slam"' in text
    assert 'DeclareLaunchArgument("map_yaml", default_value=""' in text
    assert 'include_generic_payload": "false"' in text
    assert 'loaded_product": "false"' in text
    assert 'factory_attachment": "false"' in text
    assert 'require_manipulator_stowed' in text
    assert 'ParameterValue(LaunchConfiguration("auto_start_exploration"' in text
    assert "ready" in text and "portable_exploration_readiness" in text
    assert "amr_simulation.launch.py" not in text


def test_static_localization_mode_requires_an_existing_absolute_map_yaml(tmp_path):
    launch = _load_launch()

    assert launch.PORTABLE_LOCALIZATION_MODES == ("slam", "amcl")
    assert launch.validate_localization_mode("slam") == "slam"
    assert launch.validate_localization_mode("AMCL") == "amcl"
    with pytest.raises(launch.WorldValidationError):
        launch.validate_localization_mode("unknown")
    with pytest.raises(launch.WorldValidationError):
        launch.validate_map_yaml("")

    map_yaml = tmp_path / "aws_candidate.yaml"
    map_yaml.write_text("image: aws_candidate.pgm\n")
    assert launch.validate_map_yaml(str(map_yaml)) == map_yaml.resolve()
    with pytest.raises(launch.WorldValidationError):
        launch.validate_map_yaml(str(tmp_path / "missing.yaml"))


def test_process_release_callbacks_fail_closed_on_nonzero_one_shot():
    launch = _load_launch()
    release = launch._release_one_shot([], "insertion")
    assert release(SimpleNamespace(returncode=0), None) == []
    failed = release(SimpleNamespace(returncode=2), None)
    assert len(failed) == 1
    assert isinstance(failed[0], launch.Shutdown)


@pytest.mark.parametrize(
    "script_name",
    ["portable_stow_authority.py", "portable_exploration_readiness.py"],
)
def test_portable_source_scripts_are_executable(script_name):
    mode = stat.S_IMODE((ROOT / "scripts" / script_name).stat().st_mode)
    assert mode == 0o755


def test_package_declares_direct_exploration_runtime_dependency():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {
        element.text.strip()
        for element in package.findall("exec_depend")
        if element.text and element.text.strip()
    }
    assert "amr_exploration" in dependencies


def test_causal_stage_successors_and_exact_readiness_stage_arguments_are_production_contract():
    launch = _load_launch()
    assert launch.portable_stage_successors() == (
        ("right_gripper_controller", "adapters_authority"),
        ("adapters_authority", "slam_map"),
        ("slam_map", "planner_smoother"),
        ("planner_smoother", "controller"),
        ("controller", "mission_final"),
        ("mission_final", "explorer"),
    )
    assert launch.PORTABLE_READINESS_STAGES == (
        "adapters_authority", "slam_map", "planner_smoother", "controller", "mission_final",
    )
    for stage in launch.PORTABLE_READINESS_STAGES:
        node = launch._readiness_node(stage)
        assert any(
            substitution.perform(LaunchContext()).strip().splitlines()[0] == stage
            for parameters in node._Node__parameters
            for value in parameters.values()
            if isinstance(value, tuple)
            for substitution in value
        )


def test_causal_graph_uses_lifecycle_aware_includes_and_adapter_barrier():
    launch = _load_launch()
    assert launch.PORTABLE_INCLUDED_LAUNCH_PACKAGES == (
        "amr_slam", "amr_navigation", "amr_mpc_controller", "amr_control", "amr_mission",
    )
    assert launch.PORTABLE_ADAPTER_CONFIGURE_DELAY_S == 8.0
    assert launch.PORTABLE_CONTROLLER_FREQUENCY == 5.0
    assert launch.PORTABLE_CONTROLLER_SERVICE_TIMEOUT_SEC == 60.0
    assert launch.PORTABLE_CONTROLLER_SERVICE_TIMEOUT_SEC > 0
    assert launch.PORTABLE_BASE_INPUT_TIMEOUT_MS == 3500
    assert launch.PORTABLE_BASE_INPUT_TIMEOUT_MS > 0
    assert launch.PORTABLE_BASE_GATED_COMMAND_TIMEOUT_MS == 1500
    assert launch.PORTABLE_BASE_GATED_COMMAND_TIMEOUT_MS > 0
    assert launch.PORTABLE_MAP_TIMEOUT_SEC == 12.0
    assert launch.PORTABLE_MAP_TIMEOUT_SEC > 0
    assert launch.PORTABLE_GOAL_TIMEOUT_SEC == 600.0
    assert launch.PORTABLE_GOAL_TIMEOUT_SEC > 0
    assert launch.PORTABLE_STARTUP_GRACE_SEC == 30.0
    assert launch.PORTABLE_STARTUP_GRACE_SEC > 0
    assert launch.PORTABLE_TF_TIMEOUT_SEC == 1.5
    assert launch.PORTABLE_TF_TIMEOUT_SEC > 0
    assert launch.PORTABLE_CONTROL_SOURCE_TIMEOUT_MS == 1500
    assert launch.PORTABLE_CONTROL_SOURCE_TIMEOUT_MS > 0
    assert launch.PORTABLE_MANIPULATOR_STATUS_TIMEOUT_MS == 2000
    assert launch.PORTABLE_MANIPULATOR_STATUS_TIMEOUT_MS > 0
    assert launch.PORTABLE_CONTROL_ARGUMENTS == {
        "require_manipulator_stowed": "true",
        "source_timeout_ms": "1500",
        "manipulator_status_timeout_ms": "2000",
        "configure_delay_sec": "8.0",
    }
    source = LAUNCH_PATH.read_text()
    assert "PORTABLE_CONTROLLER_SERVICE_TIMEOUT_SEC = 60.0" in source
    assert source.count('executable="portable_controller_spawner.py"') == 4
    for controller in (
        "joint_state_broadcaster",
        "arm_controller",
        "gripper_controller",
        "gripper_right_controller",
    ):
        assert (
            f'            "{controller}",\n'
            '            "--controller-manager-timeout", "30",\n'
            '            "--service-call-timeout", str(PORTABLE_CONTROLLER_SERVICE_TIMEOUT_SEC),\n'
        ) in source
    mpc_start = source.index('mpc_include = _package_launch_include(')
    mission_start = source.index('mission_include = _package_launch_include(', mpc_start)
    mpc_call = source[mpc_start:mission_start]
    assert 'arguments={"controller_frequency": str(PORTABLE_CONTROLLER_FREQUENCY)}' in mpc_call


def test_portable_base_adapter_carries_simulation_input_timeout():
    launch = _load_launch()
    source = LAUNCH_PATH.read_text()
    base_start = source.index(
        'base, (base_activate, base_configure) = _managed_node(')
    front_start = source.index(
        'front, (front_activate, front_configure) = _managed_node(', base_start)
    base_call = source[base_start:front_start]

    assert (
        '"input_timeout_ms": PORTABLE_BASE_INPUT_TIMEOUT_MS,' in base_call
        and '"gated_command_timeout_ms": PORTABLE_BASE_GATED_COMMAND_TIMEOUT_MS,' in base_call
    )
    adapter_tail_start = source.index("adapter_tail: List[object] = [")
    adapter_tail_end = source.index("adapter_initial.append", adapter_tail_start)
    adapter_tail = source[adapter_tail_start:adapter_tail_end]
    for timer in (
        """TimerAction(
            period=PORTABLE_ADAPTER_CONFIGURE_DELAY_S,
            actions=[wheel_configure],
        )""",
        """TimerAction(
            period=PORTABLE_ADAPTER_CONFIGURE_DELAY_S + 1.0,
            actions=[front_perception_configure],
        )""",
        """TimerAction(
            period=PORTABLE_ADAPTER_CONFIGURE_DELAY_S + 2.0,
            actions=[rear_perception_configure],
        )""",
        """TimerAction(
            period=PORTABLE_ADAPTER_CONFIGURE_DELAY_S + 3.0,
            actions=[health_configure],
        )""",
    ):
        assert timer in adapter_tail


def test_portable_explorer_carries_simulation_map_timeout():
    launch = _load_launch()
    source = LAUNCH_PATH.read_text()
    explorer_start = source.index(
        'explorer = Node(\n        package="amr_exploration",')
    rviz_start = source.index('    rviz_config = LaunchConfiguration(', explorer_start)
    explorer_call = source[explorer_start:rviz_start]

    assert '{"map_timeout_sec": PORTABLE_MAP_TIMEOUT_SEC}' in explorer_call
    assert '{"goal_timeout_sec": PORTABLE_GOAL_TIMEOUT_SEC}' in explorer_call
    assert '{"startup_grace_sec": PORTABLE_STARTUP_GRACE_SEC}' in explorer_call
    assert launch.PORTABLE_TF_TIMEOUT_SEC == 1.5
    assert launch.PORTABLE_TF_TIMEOUT_SEC > 0
    assert '{"tf_timeout_sec": PORTABLE_TF_TIMEOUT_SEC}' in explorer_call
    assert launch.PORTABLE_AUTHORITY_TIMEOUT_SEC == 3.0
    assert launch.PORTABLE_AUTHORITY_TIMEOUT_SEC > 0
    assert '{"authority_timeout_sec": PORTABLE_AUTHORITY_TIMEOUT_SEC}' in explorer_call


def test_each_success_callback_releases_only_its_immediate_successor_and_nonzero_shuts_down():
    launch = _load_launch()
    release = launch._release_one_shot(["immediate-successor"], "stage")
    assert release(SimpleNamespace(returncode=0), None) == ["immediate-successor"]
    failed = release(SimpleNamespace(returncode=1), None)
    assert len(failed) == 1
    assert isinstance(failed[0], launch.Shutdown)


def test_global_required_process_exit_is_fail_closed_but_respects_shutdown_and_one_shots():
    launch = _load_launch()
    expected = object()
    handler = launch._shutdown_on_required_process_exit({expected})
    callback = handler.event_handler._OnActionEventBase__on_event
    assert callback(SimpleNamespace(action=expected), SimpleNamespace(is_shutdown=False)) == []
    assert callback(SimpleNamespace(action=object()), SimpleNamespace(is_shutdown=True)) == []
    failed = callback(SimpleNamespace(action=object()), SimpleNamespace(is_shutdown=False))
    assert len(failed) == 1
    assert isinstance(failed[0], launch.Shutdown)
