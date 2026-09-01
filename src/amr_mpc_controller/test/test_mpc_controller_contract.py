from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_rpp_is_direct_and_within_simulation_limits():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/amr/localization/wheel_odometry"
    plugin = controller["FollowPath"]
    assert plugin["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
    assert "primary_controller" not in plugin
    assert plugin["desired_linear_vel"] == 0.50
    assert plugin["lookahead_dist"] == 0.60
    assert plugin["min_lookahead_dist"] == 0.30
    assert plugin["max_lookahead_dist"] == 0.90
    assert plugin["lookahead_time"] == 1.50
    assert plugin["use_velocity_scaled_lookahead_dist"] is True
    assert plugin["transform_tolerance"] == 0.30
    assert plugin["use_rotate_to_heading"] is True
    assert plugin["rotate_to_heading_angular_vel"] == 0.40
    assert plugin["max_angular_accel"] == 0.40
    assert plugin["rotate_to_heading_min_angle"] == 0.785
    assert plugin["allow_reversing"] is False
    assert plugin["use_interpolation"] is True


def test_rpp_regulation_thresholds_are_explicit_and_fail_closed():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    plugin = config["/amr/controller_server"]["ros__parameters"]["FollowPath"]
    assert plugin["min_approach_linear_velocity"] == 0.05
    assert plugin["approach_velocity_scaling_dist"] == 0.60
    assert plugin["use_regulated_linear_velocity_scaling"] is True
    assert plugin["regulated_linear_scaling_min_radius"] == 0.90
    assert plugin["regulated_linear_scaling_min_speed"] == 0.15
    assert plugin["use_cost_regulated_linear_velocity_scaling"] is True
    assert plugin["cost_scaling_dist"] > 0.0
    assert plugin["cost_scaling_gain"] > 0.0
    assert plugin["inflation_cost_scaling_factor"] > 0.0
    assert plugin["use_collision_detection"] is True
    assert plugin["max_allowed_time_to_collision_up_to_carrot"] > 0.0


def test_placement_controller_is_collision_checked_and_can_back_away():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["controller_plugins"] == ["FollowPath", "PlacementFollowPath"]
    normal = controller["FollowPath"]
    placement = controller["PlacementFollowPath"]
    assert placement["plugin"] == normal["plugin"]
    assert placement["desired_linear_vel"] == 0.10
    assert placement["use_collision_detection"] is True
    assert placement["min_approach_linear_velocity"] == 0.01
    assert placement["allow_reversing"] is True
    assert placement["use_rotate_to_heading"] is False
    assert normal["allow_reversing"] is False
    assert normal["use_rotate_to_heading"] is True


def test_retreat_goal_checker_is_xy_only_and_tight():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["goal_checker_plugins"] == [
        "goal_checker", "placement_goal_checker", "retreat_goal_checker"]
    checker = controller["retreat_goal_checker"]
    assert checker["plugin"] == "nav2_controller::PositionGoalChecker"
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == 0.01
    assert "yaw_goal_tolerance" not in checker


def test_progress_checker_counts_deliberate_diff_drive_rotation():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["progress_checker"]
    assert checker["plugin"] == "nav2_controller::PoseProgressChecker"
    assert checker["required_movement_radius"] == 0.20
    assert checker["required_movement_angle"] == 0.20
    assert checker["movement_time_allowance"] == 10.0


def test_localized_goal_window_leaves_margin_for_independent_dock_acceptance():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["goal_checker"]
    assert checker["xy_goal_tolerance"] == 0.07
    assert checker["yaw_goal_tolerance"] == 0.15
    assert controller["goal_checker_plugins"] == [
        "goal_checker", "placement_goal_checker", "retreat_goal_checker"]


def test_precise_goal_checker_is_private_and_tightens_xy_only():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["placement_goal_checker"]
    assert checker["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == 0.01
    assert checker["yaw_goal_tolerance"] == 0.15


def test_regulated_pure_pursuit_dependency_is_declared():
    package = (ROOT / "package.xml").read_text()
    assert "<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>" in package
    assert "nav2_mppi_controller" not in package
    assert "nav2_rotation_shim_controller" not in package


def test_local_costmap_uses_local_state_and_both_perception_clouds():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    costmap = config["/amr/local_costmap/local_costmap"]["ros__parameters"]
    assert costmap["global_frame"] == "odom"
    assert costmap["robot_base_frame"] == "base_footprint"
    assert costmap["inflation_layer"]["inflation_radius"] >= 0.41
    obstacle = costmap["obstacle_layer"]
    assert obstacle["front_points"]["topic"] == "/amr/perception/front_lidar/points"
    assert obstacle["rear_points"]["topic"] == "/amr/perception/rear_lidar/points"


def test_controller_output_is_internal_to_arbitration():
    launch = (ROOT / "launch" / "amr_mpc_controller.launch.py").read_text()
    assert '("cmd_vel", "/amr/mpc/cmd_vel")' in launch


def test_lifecycle_manager_starts_after_controller_construction_barrier():
    launch = (ROOT / "launch" / "amr_mpc_controller.launch.py").read_text()
    assert "controller_server = Node(" in launch
    assert "lifecycle_manager = Node(" in launch
    assert "TimerAction(period=1.0, actions=[lifecycle_manager])" in launch
