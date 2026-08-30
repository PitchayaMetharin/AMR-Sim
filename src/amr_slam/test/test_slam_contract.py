from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_online_mapping_uses_local_state_and_front_scan():
    config = yaml.safe_load((ROOT / "config" / "mapper.yaml").read_text())
    parameters = config["/amr/slam_toolbox"]["ros__parameters"]
    assert parameters["use_sim_time"] is True
    assert parameters["map_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_frame"] == "base_footprint"
    assert parameters["scan_topic"] == "/amr/sensors/front_lidar/scan"
    assert parameters["mode"] == "mapping"
    assert parameters["scan_queue_size"] == 10
    assert parameters["transform_timeout"] == 1.0
    assert parameters["min_laser_range"] == 0.2
    assert parameters["use_map_saver"] is True


def test_launch_contains_only_slam_toolbox_runtime():
    launch = (ROOT / "launch" / "amr_slam.launch.py").read_text()
    assert 'package="slam_toolbox"' in launch
    assert 'executable="async_slam_toolbox_node"' in launch
    assert "cmd_vel" not in launch
    assert "nav2" not in launch
