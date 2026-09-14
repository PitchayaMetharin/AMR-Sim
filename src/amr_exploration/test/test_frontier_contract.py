import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from frontier_algorithm import NAVIGATION_FOOTPRINT  # noqa: E402


def test_explorer_has_only_action_and_stop_boundaries():
    source = (ROOT / "scripts" / "frontier_explorer.py").read_text()
    assert os.access(ROOT / "scripts" / "frontier_explorer.py", os.X_OK)
    assert '"/map"' in source
    assert 'from nav2_msgs.msg import Costmap' in source
    assert '"/amr/global_costmap/costmap_raw"' in source
    assert "last_costmap_at" in source
    assert "costmap_frontier_candidates" in source
    assert "NAVIGATION_FOOTPRINT" in source
    assert "footprint=NAVIGATION_FOOTPRINT" in source
    assert '"/amr/mission/navigate_to_pose"' in source
    assert '"/amr/exploration/start"' in source
    assert '"/amr/exploration/stop"' in source
    assert '"/amr/exploration/status"' in source
    assert 'declare_parameter("autostart", True)' in source
    assert "cmd_vel" not in source
    assert "Twist" not in source
    assert "no_frontier_updates" in source
    assert "min_goal_distance_m" in source
    assert "STATUS_CANCELED" in source
    assert "ReentrantCallbackGroup" in source
    assert "callback_group=self.action_callback_group" in source
    assert 'message.header.frame_id != "map"' in source
    assert "last_base_status_at" in source
    assert "last_manipulator_status_at" in source
    assert 'exploration is faulted; restart the explorer' in source


def test_package_declares_runtime_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {item.text for item in package.findall("exec_depend")}
    assert {
        "diagnostic_msgs", "nav2_msgs", "nav_msgs",
        "rclpy", "std_srvs", "tf2_ros"
    } <= dependencies


def test_frontier_footprint_matches_padded_nav2_footprint():
    planner_path = ROOT.parent / "amr_navigation" / "config" / "planner.yaml"
    planner = yaml.safe_load(planner_path.read_text())
    configured = yaml.safe_load(
        planner["/amr/global_costmap/global_costmap"]["ros__parameters"][
            "footprint"])
    expected = tuple(
        (point[0] + (0.01 if point[0] >= 0.0 else -0.01),
         point[1] + (0.01 if point[1] >= 0.0 else -0.01))
        for point in configured)

    assert all(
        actual == pytest.approx(configured_point)
        for actual, configured_point
        in zip(NAVIGATION_FOOTPRINT, expected))
