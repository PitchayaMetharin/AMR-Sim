"""Acceptance wiring checks for front + rear LiDAR in SLAM and AMCL.

Written before the implementation; the implementer must not edit this file.
Every world goes through these launches, so SLAM and AMCL use both LiDARs
on any loaded world or saved map.
"""
import json
from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
MERGED = "/amr/sensors/merged_lidar/scan"
FRONT = "/amr/sensors/front_lidar/scan"
NODE = "lidar_scan_merger_node"


def _read(relative):
    return (SRC / relative).read_text()


def test_merger_node_is_built_installed_and_tested():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert re.search(rf"add_executable\(\s*{NODE}\b", cmake)
    assert re.search(rf"install\(TARGETS[^)]*\b{NODE}\b", cmake)
    assert re.search(r"ament_add_gtest\(\s*test_scan_merge\s+test/test_scan_merge\.cpp", cmake)
    assert re.search(
        r"ament_add_pytest_test\(\s*\w+\s+test/test_dual_lidar_wiring\.py", cmake)
    package = (ROOT / "package.xml").read_text()
    assert "<depend>tf2_ros</depend>" in package


def test_merger_node_merges_both_scans_with_fail_closed_freshness():
    source = (ROOT / "src" / f"{NODE}.cpp").read_text()
    assert "LifecycleNode" in source
    assert '"/amr/sensors/front_lidar/scan"' in source
    assert '"/amr/sensors/rear_lidar/scan"' in source
    assert f'"{MERGED}"' in source
    assert '"base_footprint"' in source
    assert '"odom"' in source
    assert "lookupTransform" in source
    assert "merge_scans(" in source
    assert "stamps_usable(" in source
    assert "amr_interfaces::qos::sensor()" in source
    # Perception only: no TF ownership and no motion authority.
    assert "TransformBroadcaster" not in source
    assert "sendTransform" not in source
    assert "twist" not in source.lower()
    assert "cmd_vel" not in source


def test_merger_runs_wherever_slam_or_amcl_runs():
    perception_launch = (ROOT / "launch" / "amr_perception.launch.py").read_text()
    assert f'"{NODE}"' in perception_launch
    portable = _read("amr_simulation/launch/portable_exploration.launch.py")
    assert re.search(rf'_managed_node\(\s*"amr_perception",\s*"{NODE}"', portable)
    # Factory mapping/localization reach the merger through amr_perception.launch.py.
    factory_localization = _read("amr_factory/launch/factory_localization.launch.py")
    assert '"amr_perception.launch.py"' in factory_localization
    factory_mapping = _read("amr_factory/launch/factory_mapping.launch.py")
    assert '"factory_localization.launch.py"' in factory_mapping


def test_slam_reads_the_merged_scan():
    config = yaml.safe_load(_read("amr_slam/config/mapper.yaml"))
    parameters = config["/amr/slam_toolbox"]["ros__parameters"]
    assert parameters["scan_topic"] == MERGED
    assert parameters["base_frame"] == "base_footprint"
    assert parameters["max_laser_range"] == 20.0


def test_every_amcl_reads_the_merged_scan():
    factory_amcl = yaml.safe_load(_read("amr_factory/config/amcl.yaml"))
    assert factory_amcl["/amr/amcl"]["ros__parameters"]["scan_topic"] == MERGED
    for relative in (
            "amr_factory/launch/factory_localization.launch.py",
            "amr_simulation/launch/portable_exploration.launch.py"):
        launch = _read(relative)
        assert f'("scan", "{MERGED}")' in launch, relative
        assert f'("scan", "{FRONT}")' not in launch, relative
    portable = _read("amr_simulation/launch/portable_exploration.launch.py")
    assert f'"scan_topic": "{MERGED}"' in portable
    assert f'"scan_topic": "{FRONT}"' not in portable


def test_merged_scan_has_one_registered_publisher():
    contract = json.loads(_read("amr_bringup/config/interface_ownership.yaml"))
    assert contract["topics"][MERGED] == {
        "type": "sensor_msgs/msg/LaserScan",
        "publisher": f"amr_perception/{NODE}",
    }
    # The per-sensor topics stay separate and unchanged.
    assert contract["topics"][FRONT]["publisher"] == (
        "amr_sensor_adapters/front_lidar_adapter_node")
    assert contract["topics"]["/amr/sensors/rear_lidar/scan"]["publisher"] == (
        "amr_sensor_adapters/rear_lidar_adapter_node")
