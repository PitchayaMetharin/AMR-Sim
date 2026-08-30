from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_control_lifecycle_registers_activation_before_configuration():
    launch = (ROOT / "launch" / "amr_control.launch.py").read_text()
    managed_start = launch.index("def managed_node")
    return_order = launch.index("return node, activate, configure", managed_start)
    assert "return node, configure, activate" not in launch[managed_start:return_order]
    assert launch.index("activate = RegisterEventHandler", managed_start) < return_order


def test_arbitration_owns_constraints_and_stamped_command():
    source = (ROOT / "src" / "command_arbitration_node.cpp").read_text()
    config = yaml.safe_load((ROOT / "config" / "control.yaml").read_text())
    parameters = config["/amr/command_arbitration_node"]["ros__parameters"]
    assert '"/amr/mpc/cmd_vel"' in source
    assert '"/amr/control/cmd_vel"' in source
    assert "std::clamp" in source
    assert "steady_clock" in source
    assert "qos::nav2_command_input()" in source
    assert parameters["source_timeout_ms"] == 200
    assert parameters["require_manipulator_stowed"] is False
    assert parameters["manipulator_status_timeout_ms"] == 200
    assert parameters["max_linear_velocity"] <= 0.5
    assert parameters["max_angular_velocity"] <= 0.4

def test_control_does_not_publish_to_simulation():
    source = (ROOT / "src" / "command_arbitration_node.cpp").read_text()
    assert "/amr/simulation/base/cmd_vel" not in source


def test_factory_interlock_uses_receive_time_and_fail_closed_semantics():
    source = (ROOT / "src" / "command_arbitration_node.cpp").read_text()
    assert '"/amr/manipulation/status"' in source
    assert "valid_manipulator_semantics" in source
    assert "last_manipulator_status_" in source
    assert "STOWED_EMPTY" in source
    assert "STOWED_LOADED" in source


def test_dock_egress_is_back_up_owned_by_the_arbitrator_and_configured():
    source = (ROOT / "src" / "command_arbitration_node.cpp").read_text()
    config = yaml.safe_load((ROOT / "config" / "control.yaml").read_text())
    parameters = config["/amr/command_arbitration_node"]["ros__parameters"]
    assert 'nav2_msgs/action/back_up.hpp' in source
    assert '"/amr/control/dock_egress"' in source
    assert "STOWED_LOADED" in source
    assert "rear_lidar/scan" in source
    assert "/amr/localization/odometry" in source
    assert "egress_max_distance_m" in source
    assert parameters["egress_max_distance_m"] == 0.50
    assert parameters["egress_max_speed_mps"] == 0.10
    assert parameters["egress_time_limit_s"] == 60.0
    assert parameters["egress_scan_timeout_ms"] == 1000
    assert parameters["egress_clearance_m"] == 0.05
