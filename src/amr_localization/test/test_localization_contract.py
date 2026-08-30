import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def parameters():
    config = yaml.safe_load((ROOT / "config" / "ekf.yaml").read_text())
    return config["/amr/ekf_filter_node"]["ros__parameters"]


def test_config_key_matches_namespaced_launch_node():
    config = yaml.safe_load((ROOT / "config" / "ekf.yaml").read_text())
    assert set(config) == {"/amr/ekf_filter_node"}


def test_ekf_is_planar_and_owns_only_local_transform():
    config = parameters()
    assert config["two_d_mode"] is True
    assert config["publish_tf"] is True
    assert config["reset_on_time_jump"] is True
    assert config["world_frame"] == "odom"
    assert config["odom_frame"] == "odom"
    assert config["base_link_frame"] == "base_footprint"


def test_ekf_fuses_wheel_velocity_and_imu_yaw_and_yaw_rate():
    config = parameters()
    assert config["odom0"] == "/amr/localization/wheel_odometry"
    assert config["imu0"] == "/amr/sensors/imu/data_raw"
    assert config["odom0_config"][6:12] == [
        True, True, False, False, False, True]
    assert config["imu0_config"][5] is True
    assert config["imu0_config"][11] is True


def test_launch_exposes_filtered_odometry_without_global_mapping():
    launch = (ROOT / "launch" / "amr_localization.launch.py").read_text()
    assert 'executable="wheel_odometry_node"' in launch
    assert 'executable="ekf_node"' in launch
    assert '"/amr/localization/odometry"' in launch
    assert "slam_toolbox" not in launch
    assert "map_server" not in launch


def test_wheel_odometry_geometry_matches_gazebo_diff_drive():
    source = (ROOT / "src" / "wheel_odometry_node.cpp").read_text()
    launch = (ROOT / "launch" / "amr_localization.launch.py").read_text()
    assert 'declare_parameter("wheel_radius", 0.1128)' in source
    assert 'declare_parameter("wheel_separation", 0.566)' in source
    assert '"wheel_radius": 0.1128' in launch
    assert '"wheel_separation": 0.566' in launch

    description = ROOT.parent / "amr_description" / "urdf" / "amr.urdf.xacro"
    robot = ET.fromstring(subprocess.check_output(["xacro", str(description)], text=True))
    plugin = robot.find("./gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(plugin.find("wheel_radius").text) == 0.1128
    assert float(plugin.find("wheel_separation").text) == 0.566


def test_wheel_odometry_publishes_reliable_state_for_nav2_consumers():
    source = (ROOT / "src" / "wheel_odometry_node.cpp").read_text()
    assert "create_publisher<nav_msgs::msg::Odometry>" in source
    assert "output_topic_, amr_interfaces::qos::state()" in source


def test_live_acceptance_has_quantitative_nominal_bounds():
    script = (ROOT / "scripts" / "localization_acceptance.py").read_text()
    assert "POSITION_LIMIT = 0.03" in script
    assert "YAW_LIMIT = 0.04" in script
    assert "STATIC_LIMIT = 0.01" in script
    assert '"/amr/simulation/ground_truth/pose"' in script
    assert '"/amr/localization/odometry"' in script


def test_authority_bypassing_acceptance_is_not_runtime_installed():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "localization_acceptance.py" not in cmake
