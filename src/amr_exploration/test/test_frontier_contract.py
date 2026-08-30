from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_explorer_has_only_action_and_stop_boundaries():
    source = (ROOT / "scripts" / "frontier_explorer.py").read_text()
    assert '"/map"' in source
    assert '"/amr/mission/navigate_to_pose"' in source
    assert '"/amr/exploration/stop"' in source
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
    assert {"nav2_msgs", "nav_msgs", "rclpy", "std_srvs", "tf2_ros"} <= dependencies
