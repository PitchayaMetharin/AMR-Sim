import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_world_loads_required_systems_and_physics():
    world = ET.parse(ROOT / "worlds" / "amr_world.sdf").getroot().find("world")
    plugins = {plugin.attrib["filename"] for plugin in world.findall("plugin")}
    assert {
        "gz-sim-physics-system",
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "gz-sim-sensors-system",
        "gz-sim-imu-system",
    } <= plugins
    assert float(world.find("./physics/max_step_size").text) == 0.001
    fixture = world.find("./model[@name='mapping_fixture']")
    assert fixture is not None
    assert len(fixture.findall("./link/collision")) >= 4


def test_launch_supports_headless_and_correct_point_cloud_sources():
    launch = (ROOT / "launch" / "amr_simulation.launch.py").read_text()
    assert 'DeclareLaunchArgument("headless"' in launch
    assert 'arguments=["-name", "amr", "-param", "robot_description"]' in launch
    assert "front_lidar/scan/points@sensor_msgs/msg/PointCloud2" in launch
    assert "rear_lidar/scan/points@sensor_msgs/msg/PointCloud2" in launch
    assert 'front_lidar/scan/points", "/amr/simulation/sensors/front_lidar/points' in launch
    assert 'rear_lidar/scan/points", "/amr/simulation/sensors/rear_lidar/points' in launch
    assert '"amr_localization.launch.py"' in launch
    assert '"amr_perception.launch.py"' in launch
    assert '"amr_navigation.launch.py"' in launch
    assert '"amr_mpc_controller.launch.py"' in launch
    assert '"amr_control.launch.py"' in launch
    assert '"amr_mission.launch.py"' in launch
    assert '"amr_health", "health_supervisor_node"' in launch
    assert '"/model/amr/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"' in launch
    assert '"/model/amr/pose", "/amr/simulation/ground_truth/pose"' in launch


def test_plant_has_an_independent_command_watchdog():
    description = (
        Path(__file__).resolve().parents[2]
        / "amr_description"
        / "urdf"
        / "amr.urdf.xacro"
    ).read_text()
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "amr-command-watchdog-system" in description
    assert "<command_timeout_ms>200</command_timeout_ms>" in description
    assert "test_command_watchdog" in cmake
