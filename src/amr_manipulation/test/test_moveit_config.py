import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_moveit_uses_required_group_planner_and_execution_limits():
    assert yaml.safe_load((ROOT / "config" / "joint_limits.yaml").read_text()) == {
        "joint_limits": {}}
    kinematics = yaml.safe_load((ROOT / "config" / "kinematics.yaml").read_text())
    assert kinematics["manipulator"]["kinematics_solver"] == (
        "kdl_kinematics_plugin/KDLKinematicsPlugin")

    ompl = yaml.safe_load((ROOT / "config" / "ompl_planning.yaml").read_text())
    assert ompl["planning_plugin"] == "ompl_interface/OMPLPlanner"
    assert ompl["manipulator"]["default_planner_config"] == (
        "RRTConnectkConfigDefault")
    assert ompl["planner_configs"]["RRTConnectkConfigDefault"]["type"] == (
        "geometric::RRTConnect")
    assert ompl["planner_configs"]["RRTConnectkConfigDefault"][
        "longest_valid_segment_fraction"] == 0.001

    controllers = yaml.safe_load(
        (ROOT / "config" / "moveit_controllers.yaml").read_text())
    manager = controllers["moveit_simple_controller_manager"]
    assert manager["arm_controller"]["type"] == "FollowJointTrajectory"
    assert manager["gripper_controller"]["type"] == "GripperCommand"
    assert controllers["trajectory_execution"] == {
        "allowed_execution_duration_scaling": 1.2,
        "allowed_goal_duration_margin": 1.0,
        "allowed_start_tolerance": 0.01,
    }


def test_moveit_launch_sets_factory_model_and_publishes_descriptions():
    launch = (ROOT / "launch" / "move_group.launch.py").read_text()
    assert '"factory_attachment": "true"' in launch
    assert "publish_robot_description=True" in launch
    assert "publish_robot_description_semantic=True" in launch
    assert 'default_planning_pipeline="ompl"' in launch
    assert '("joint_states", "/amr/base/joint_states")' in launch

    acceptance = (ROOT / "launch" / "gate6_empty_motion.launch.py").read_text()
    assert '("joint_states", "/amr/base/joint_states")' in acceptance
    source = (ROOT / "src" / "gate6_empty_motion.cpp").read_text()
    assert 'MoveGroupInterface arm(node, "manipulator")' in source

    mass_source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    assert 'nav2_msgs/action/back_up.hpp' in mass_source
    assert '"/amr/control/dock_egress"' in mass_source
    assert "staging_waypoints, 0.005, 0.0, staging_trajectory, true" in mass_source
    bootstrap = mass_source.index("verify_attachment_bootstrap(5s)")
    reference = mass_source.index("capture_reference_evidence(3s)")
    gripper = mass_source.index("command_gripper(node, 0.035)")
    assert bootstrap < reference < gripper
    assert "request_and_confirm_initial_detachment" not in mass_source
    assert 'unsafe wrist-flipped staging branch rejected' in mass_source
    assert (
        "pregrasp.position.x = 0.85;\n"
        "    pregrasp.position.y = pickup_product_lateral;\n"
        "    pregrasp.position.z = 1.00;"
    ) in mass_source
    assert 'arm.setJointValueTarget(pregrasp, "gripper_tcp")' not in mass_source
    assert "pregrasp_seed{" in mass_source
    assert "-0.000032311, -0.760907950, 0.661511204" in mass_source
    assert "0.000037514, 0.099207379, -0.000017083" in mass_source
    pregrasp_seed = mass_source.index("pregrasp_seed{")
    pregrasp_state = mass_source.index(
        "auto pregrasp_ik_state = arm.getCurrentState(3.0)")
    pregrasp_group = mass_source.index("expected_pregrasp_joint_names")
    pregrasp_order = mass_source.index(
        "pregrasp_manipulator_group->getVariableNames() != expected_pregrasp_joint_names")
    pregrasp_count = mass_source.index(
        "pregrasp_manipulator_group->getVariableCount() != pregrasp_seed.size()")
    pregrasp_finite_seed = mass_source.index(
        "finite_pregrasp_joint_values(pregrasp_seed)")
    pregrasp_set_seed = mass_source.index(
        "setJointGroupPositions(\n      pregrasp_manipulator_group, pregrasp_seed)")
    pregrasp_ik = mass_source.index(
        'pregrasp_ik_state->setFromIK(\n        pregrasp_manipulator_group, pregrasp, "gripper_tcp", 0.5)')
    pregrasp_bounds = mass_source.index(
        "pregrasp_ik_state->satisfiesBounds(pregrasp_manipulator_group)")
    pregrasp_copy = mass_source.index(
        "copyJointGroupPositions(\n      pregrasp_manipulator_group, pregrasp_ik_solution)")
    pregrasp_finite_solution = mass_source.index(
        "finite_pregrasp_joint_values(pregrasp_ik_solution)")
    pregrasp_start = mass_source.index("arm.setStartStateToCurrentState();", pregrasp_copy)
    pregrasp_target = mass_source.index(
        "arm.setJointValueTarget(pregrasp_ik_solution)")
    pregrasp_ompl = mass_source.index("arm.plan(pregrasp_plan)")
    assert pregrasp_seed < pregrasp_state < pregrasp_group
    assert pregrasp_group < pregrasp_order < pregrasp_count
    assert pregrasp_count < pregrasp_finite_seed < pregrasp_set_seed
    assert pregrasp_set_seed < pregrasp_ik < pregrasp_bounds < pregrasp_copy
    assert pregrasp_copy < pregrasp_finite_solution < pregrasp_start
    assert pregrasp_start < pregrasp_target < pregrasp_ompl
    assert 'unsafe wrist-flipped pre-grasp branch rejected' in mass_source
    assert "command_gripper(node, 0.020)" in mass_source
    command_start = mass_source.index("bool command_gripper")
    command_end = mass_source.index(
        "std::array<double, 3> map_point_to_base", command_start)
    command_source = mass_source[command_start:command_end]
    assert '"/gripper_controller/gripper_cmd"' in command_source
    assert '"/gripper_right_controller/gripper_cmd"' in command_source
    left_send = command_source.index(
        "auto left_sent = left_client->async_send_goal(left_goal);")
    right_send = command_source.index(
        "auto right_sent = right_client->async_send_goal(right_goal);")
    acceptance_wait = command_source.index(
        "auto left_acceptance = std::async(std::launch::async", right_send)
    assert left_send < right_send < acceptance_wait
    assert command_source.count("async_send_goal") == 2
    assert "left_sent.wait_for(3s)" in command_source
    assert "right_sent.wait_for(3s)" in command_source
    assert "auto left_goal_handle = left_acceptance_ready ? left_sent.get() : nullptr;" in command_source
    assert "auto right_goal_handle = right_acceptance_ready ? right_sent.get() : nullptr;" in command_source
    acceptance_failure = command_source.index(
        "if (!left_acceptance_ready || !right_acceptance_ready ||")
    partial_left_cancel = command_source.index(
        'cancel_accepted_goal("left", left_client, left_goal_handle, left_result)',
        acceptance_failure)
    partial_right_cancel = command_source.index(
        'cancel_accepted_goal("right", right_client, right_goal_handle, right_result)',
        partial_left_cancel)
    partial_return = command_source.index("return false;", partial_right_cancel)
    left_guard = command_source.index("if (left_goal_handle)", acceptance_failure)
    right_guard = command_source.index("if (right_goal_handle)", left_guard)
    assert acceptance_failure < left_guard < partial_left_cancel
    assert partial_left_cancel < right_guard < partial_right_cancel < partial_return
    result_futures = command_source.index(
        "auto left_result = left_client->async_get_result(left_goal_handle);", partial_return)
    right_result_future = command_source.index(
        "auto right_result = right_client->async_get_result(right_goal_handle);", result_futures)
    assert right_send < result_futures < right_result_future
    cancel_helper = command_source.index("const auto cancel_accepted_goal")
    cancel_goal = command_source.index(
        "async_cancel_goal(goal_handle)", cancel_helper)
    cancel_wait = command_source.index("cancel.wait_for(3s)", cancel_goal)
    terminal_wait = command_source.index("result.wait_for(3s)", cancel_wait)
    terminal_code = command_source.index(
        "terminal.code != rclcpp_action::ResultCode::CANCELED", terminal_wait)
    assert cancel_helper < cancel_goal < cancel_wait < terminal_wait < terminal_code
    assert "response->goals_canceling" in command_source
    assert "goal_info.goal_id.uuid == goal_id" in command_source
    result_timeout = command_source.index("result.wait_for(30s)")
    timeout_cancel = command_source.index(
        "cancel_accepted_goal(side, client, goal_handle, result)", result_timeout)
    timeout_return = command_source.index("return false;", timeout_cancel)
    assert result_timeout < timeout_cancel < timeout_return
    result_code_check = command_source.index(
        "wrapped.code != rclcpp_action::ResultCode::SUCCEEDED")
    result_pointer_check = command_source.index("!wrapped.result")
    result_flags = command_source.index("wrapped.result->reached_goal")
    assert max(result_code_check, result_pointer_check) < result_flags
    assert "wrapped.result->reached_goal || wrapped.result->stalled" in command_source
    assert "const bool left_ok = left_completion.get();" in command_source
    assert "const bool right_ok = right_completion.get();" in command_source
    assert "return left_ok && right_ok;" in command_source
    assert "left >= threshold && right >= threshold" in mass_source
    close = mass_source.index("command_gripper(node, 0.020)")
    finger_positions = mass_source.index(
        "gripper_positions_above(0.020, 3s)", close)
    bilateral_contact = mass_source.index(
        "wait_for_bilateral_contact(3s)", finger_positions)
    assert close < finger_positions < bilateral_contact
    handle_add = mass_source.index('"pickup_handle", {0.04, 0.10, 0.05}')
    pregrasp_execute = mass_source.index('arm.execute(pregrasp_plan)')
    handle_remove = mass_source.index(
        "pickup_handle.operation = moveit_msgs::msg::CollisionObject::REMOVE")
    pregrasp_cartesian = mass_source.index("arm.computeCartesianPath")
    approach = mass_source.index("arm.computeCartesianPath", handle_remove)
    assert handle_add < pregrasp_cartesian < pregrasp_execute
    assert handle_add < pregrasp_execute < handle_remove < approach
    attach_scene = mass_source.index("scene.applyAttachedCollisionObject(attached)")
    allow_support = mass_source.index("set_pickup_support_collision(true)")
    lift_checkpoint = mass_source.index("lift_checkpoint = grasp")
    clearance_retreat = mass_source.index("clearance_retreat = pregrasp")
    retreat_waypoints = mass_source.index(
        "retreat_waypoints{\n        lift_checkpoint, clearance_retreat}")
    retreat_path = mass_source.index(
        "retreat_waypoints, 0.005, 0.0, retreat_trajectory, true")
    retreat_execute = mass_source.index("arm.execute(retreat_plan)")
    restore_support = mass_source.index(
        "set_pickup_support_collision(false)", retreat_execute)
    validity_service = mass_source.index('"/check_state_validity"')
    validity_helper = mass_source.index("const auto validate_state")
    validity_diff = mass_source.index("request->robot_state.is_diff = true")
    validity_contacts = mass_source.index("response->contacts")
    validity_required = mass_source.index("payload-aware state validity failed")
    payload_proof = mass_source.index("planning_scene_attached_object_proof(true)")
    loaded_stow = mass_source.index("arm.plan(stow_plan)")
    lower_execute = mass_source.index("arm.execute(lower_plan)")
    assert attach_scene < allow_support < lift_checkpoint < clearance_retreat
    assert clearance_retreat < retreat_waypoints < retreat_path < retreat_execute
    assert retreat_execute < restore_support < validity_service
    assert validity_service < validity_helper < validity_diff < validity_contacts < validity_required
    assert validity_helper < loaded_stow < payload_proof < lower_execute
    assert "retreat_waypoints" in mass_source
    assert mass_source[attach_scene:validity_service].count(
        "retreat_waypoints, 0.005, 0.0, retreat_trajectory, true") == 1
    assert "latest_product_pose(retreat_product_pose)" in mass_source
    assert "native_attachment_state_is(\"attached\")" in mass_source
    assert '"/amr/mission/navigate_to_pose_precise"' not in mass_source
    assert "precise_navigation_client_" not in mass_source
    assert "bool precise" not in mass_source
    assert "precise ?" not in mass_source
    assert "navigation_client_" in mass_source
    assert 'ensure_entry("held_product")' in mass_source
    assert 'ensure_entry("pickup_pedestal")' in mass_source
    negative_check = mass_source.index("Out-of-dispatch detachment rejection")
    egress_call = mass_source.index("node->dock_egress(65s)")
    pickup_reverse_geometry = mass_source.index(
        "dock_to_egress_dx", egress_call)
    pickup_reverse_distance = mass_source.index(
        "pickup_approach_distance = std::hypot", pickup_reverse_geometry)
    pickup_reverse = mass_source.index(
        'node->bounded_reverse(\n        pickup_approach_distance, "Pickup approach reverse", 65s)',
        pickup_reverse_distance)
    pickup_reverse_attachment = mass_source.index(
        "attachment proof failed after pickup station reverse", pickup_reverse)
    pickup_navigation = mass_source.index(
        "node->navigate_to(product.pickup_station, 120s)", pickup_reverse_attachment)
    pickup_achieved = mass_source.index(
        "pickup_station_achieved", pickup_navigation)
    pickup_xy_gate = mass_source.index(
        "pickup_station_xy_error", pickup_achieved)
    pickup_yaw_gate = mass_source.index(
        "pickup_station_yaw_error", pickup_xy_gate)
    assert negative_check < egress_call < pickup_reverse_geometry
    assert pickup_reverse_geometry < pickup_reverse_distance < pickup_reverse
    assert pickup_reverse < pickup_reverse_attachment < pickup_navigation
    assert pickup_navigation < pickup_achieved < pickup_xy_gate < pickup_yaw_gate
    dispatch_translation_start = mass_source.index(
        "dispatch_translation_start", pickup_navigation)
    dispatch_translation_bearing = mass_source.index(
        "dispatch_translation_heading = std::atan2", dispatch_translation_start)
    dispatch_translation_navigation = mass_source.index(
        "node->navigate_to(dispatch_translation_target, 120s)",
        dispatch_translation_bearing)
    translation_attachment = mass_source.index(
        "attachment proof failed after dispatch approach translation",
        dispatch_translation_navigation)
    dispatch_heading_start = mass_source.index(
        "dispatch_heading_start", translation_attachment)
    dispatch_heading_navigation = mass_source.index(
        "node->navigate_to(dispatch_heading_target, 120s)", dispatch_heading_start)
    heading_attachment = mass_source.index(
        "attachment proof failed after dispatch approach heading",
        dispatch_heading_navigation)
    assert pickup_yaw_gate < dispatch_translation_start < dispatch_translation_bearing
    assert dispatch_translation_bearing < dispatch_translation_navigation < translation_attachment
    assert translation_attachment < dispatch_heading_start < dispatch_heading_navigation
    assert dispatch_heading_navigation < heading_attachment
    assert "product.dispatch_approach[0]" in mass_source[dispatch_translation_start:dispatch_translation_navigation]
    assert "product.dispatch_approach[1]" in mass_source[dispatch_translation_start:dispatch_translation_navigation]
    assert "product.dispatch_approach[2]" in mass_source[dispatch_heading_start:dispatch_heading_navigation]
    assert "std::atan2" in mass_source[dispatch_translation_start:dispatch_translation_navigation]
    egress_source_start = mass_source.index("bool bounded_reverse")
    egress_source_end = mass_source.index(
        "bool request_and_confirm_attachment", egress_source_start)
    egress_source = mass_source[egress_source_start:egress_source_end]
    assert "goal.target.x = distance" in egress_source
    assert "goal.speed = static_cast<float>(product_.pickup_egress_speed_mps)" in egress_source
    assert "product_.pickup_egress_time_limit_s" in egress_source
    assert "product_.pickup_egress_max_distance_m" in egress_source
    assert "result.wait_for(client_timeout)" in egress_source
    assert "async_cancel_goal(goal_handle)" in egress_source
    assert "response->goals_canceling" in egress_source
    assert "terminal.code != rclcpp_action::ResultCode::CANCELED" in egress_source
    assert "bool dock_egress(std::chrono::seconds client_timeout)" in egress_source

    dock_bias_ground_truth = mass_source.index(
        "geometry_msgs::msg::PoseStamped dispatch_dock_bias_ground_truth",
        heading_attachment)
    dock_bias_localized = mass_source.index(
        "geometry_msgs::msg::PoseStamped dispatch_dock_bias_localized",
        dock_bias_ground_truth)
    dock_bias_x = mass_source.index(
        "dispatch_dock_bias_x =", dock_bias_localized)
    dock_bias_y = mass_source.index(
        "dispatch_dock_bias_y =", dock_bias_x)
    dock_bias_finite = mass_source.index(
        "std::isfinite(dispatch_dock_bias_x)", dock_bias_y)
    dock_target = mass_source.index(
        "dispatch_dock_corrected_target{", dock_bias_finite)
    dock_navigation = mass_source.index(
        "node->navigate_to(dispatch_dock_corrected_target, 120s)", dock_target)
    dock_attachment = mass_source.index(
        "attachment proof failed after dispatch dock", dock_navigation)
    registered_dock_check = mass_source.index(
        "node->dock_pose_within_tolerance(5s)", dock_attachment)
    alignment_goal = mass_source.index("placement_alignment{")
    alignment_segment_navigation = mass_source.index(
        "node->navigate_to(segment_target, 120s)")
    alignment_navigation = mass_source.index(
        "node->navigate_to(segment_target, 120s)")
    alignment_finite = mass_source.index("alignment_geometry_finite")
    alignment_bound = mass_source.index(
        "alignment_displacement > kMaxPlacementAlignmentTotalDisplacement")
    before_alignment_guard = mass_source.index(
        'if (!product_attached || !node->native_attachment_state_is("attached"))',
        alignment_bound)
    before_alignment_attachment = mass_source.index(
        "loaded-stowed attachment proof failed before placement alignment")
    after_alignment_attachment = mass_source.index(
        "attachment proof failed after placement alignment")
    after_alignment_permission = mass_source.index(
        "require_motion_permission()", alignment_navigation)
    after_alignment_pose = mass_source.index(
        "latest_robot_pose(robot_pose)", after_alignment_permission)
    release_geometry = mass_source.index("release_product_map", after_alignment_pose)
    assert pickup_yaw_gate < dispatch_translation_start
    assert heading_attachment < dock_bias_ground_truth < dock_bias_localized
    assert dock_bias_localized < dock_bias_x < dock_bias_y < dock_bias_finite
    assert dock_bias_finite < dock_target < dock_navigation
    assert dock_navigation < dock_attachment < registered_dock_check < alignment_goal
    assert alignment_goal < alignment_finite < alignment_bound
    assert loaded_stow < alignment_bound < before_alignment_guard < before_alignment_attachment < alignment_segment_navigation
    assert alignment_segment_navigation < after_alignment_attachment
    assert after_alignment_attachment < after_alignment_permission < after_alignment_pose
    assert after_alignment_pose < release_geometry
    assert "product.dispatch_slots.at(product.selected_slot_index)" in mass_source
    assert "node->navigate_to(product.pickup_station, 120s)" in mass_source
    assert "pickup_station_bearing_target" not in mass_source
    assert "pickup_station_heading_target" not in mass_source
    assert "pickup_travel_bearing" not in mass_source
    assert "pickup station reverse geometry was invalid" in mass_source
    assert "pickup_approach_distance > product.pickup_egress_max_distance_m" in mass_source
    assert "pickup_station_xy_error > 0.07" in mass_source
    assert "pickup_station_yaw_error > 0.15" in mass_source
    assert "node->navigate_to(dispatch_translation_target, 120s)" in mass_source
    assert "node->navigate_to(dispatch_heading_target, 120s)" in mass_source
    assert mass_source.count(
        "node->navigate_to(dispatch_dock_corrected_target, 120s)") == 1
    assert "node->navigate_to(product.dispatch_dock, 120s)" not in mass_source
    assert "node->navigate_to(product.dispatch_dock, 120s, true)" not in mass_source
    assert "product.dispatch_dock[0] - dispatch_dock_bias_x" in mass_source
    assert "product.dispatch_dock[1] - dispatch_dock_bias_y" in mass_source
    assert "fresh dispatch dock bias evidence was unavailable" in mass_source
    assert "dispatch dock localization bias was non-finite" in mass_source
    dock_bias_source = mass_source[dock_bias_ground_truth:dock_navigation]
    assert "latest_robot_pose(dispatch_dock_bias_ground_truth)" in dock_bias_source
    assert "latest_navigation_feedback_pose(dispatch_dock_bias_localized)" in dock_bias_source
    assert "dispatch_dock_translation" not in mass_source
    assert "dispatch_dock_heading" not in mass_source
    assert "node->navigate_to(segment_target, 120s, true)" not in mass_source
    assert "node->navigate_to(final_heading_target, 120s, true)" not in mass_source
    assert "kDockPositionTolerance = 0.155" in mass_source
    assert "kDockYawTolerance = 0.15" in mass_source
    terminal_log_start = mass_source.index("void log_navigation_terminal")
    terminal_log_end = mass_source.index("bool cancel_navigation_goal", terminal_log_start)
    terminal_log = mass_source[terminal_log_start:terminal_log_end]
    assert "ground_truth=(x=%.3f, y=%.3f, yaw=%.3f)" in terminal_log
    assert "ground_truth_yaw" in terminal_log
    assert "ground_truth.pose.position.z" not in terminal_log
    assert "const double dispatch_yaw = product.dispatch_dock[2]" in mass_source
    assert "std::cos(dispatch_yaw)" in mass_source
    assert "std::sin(dispatch_yaw)" in mass_source
    assert "std::isfinite(placement_alignment[0])" in mass_source
    assert "std::isfinite(placement_alignment[1])" in mass_source
    assert "std::isfinite(placement_alignment[2])" in mass_source
    assert "kMaxPlacementAlignmentSegmentDisplacement = 0.15" in mass_source
    assert "kMaxPlacementCommandDisplacement" in mass_source
    assert "kMaxPlacementAlignmentSegmentDisplacement - kMaxPlacementAlignmentPositionError" in mass_source
    assert "kMaxPlacementAlignmentTotalDisplacement = 0.35" in mass_source
    assert "kMaxPlacementAlignmentPositionError = 0.07" in mass_source
    assert "kMaxPlacementAlignmentYawError = 0.15" in mass_source
    assert "kFinalHeadingGoalMargin = 0.03" in mass_source
    assert "kMaxPlacementReleaseRadius = 0.785" in mass_source
    assert "achieved_alignment_displacement > kMaxPlacementAlignmentTotalDisplacement" in mass_source
    assert "fresh product attachment evidence failed after placement alignment" in mass_source
    assert "measured_attachment_evidence" in mass_source[alignment_navigation:release_geometry]
    assert "kDesiredSlotBaseX = 0.520000000" in mass_source
    assert "kDesiredSlotBaseY = -0.580000000" in mass_source
    assert "kPlacementReachReserve = 0.005" in mass_source
    assert "kDesiredSlotBaseRadius =" in mass_source
    assert "kMaxPlacementReleaseRadius - kMaxPlacementAlignmentPositionError" in mass_source
    assert "desired_slot_direction_radius =" in mass_source
    assert "const double desired_slot_base_radius = product102_center_slot ?" in mass_source
    assert "desired_slot_scale = desired_slot_base_radius / desired_slot_direction_radius" in mass_source
    assert "desired_slot_base_x = desired_slot_direction_x * desired_slot_scale" in mass_source
    assert "desired_slot_base_y = desired_slot_direction_y * desired_slot_scale" in mass_source
    assert "desired placement stance radius was invalid" in mass_source
    assert "desired placement stance scaling was non-finite" in mass_source
    assert "placement_alignment_physical" in mass_source
    assert "placement_alignment" in mass_source
    assert "alignment_segments" in mass_source
    assert "segment_heading = std::atan2" in mass_source
    assert "segment_target[2] = segment_heading" in mass_source
    assert "achieved_segment_displacement > kMaxPlacementAlignmentSegmentDisplacement" in mass_source
    assert "latest_navigation_feedback_pose" in mass_source
    assert "attachment proof failed during placement alignment" in mass_source
    assert "final_heading_target" in mass_source
    assert "fresh final heading bias evidence was unavailable" in mass_source
    assert "final_heading_bias_x" in mass_source
    assert "final_heading_bias_y" in mass_source
    assert "final_heading_bias_yaw" in mass_source
    assert "final_heading_bias_delta > kMaxPlacementAlignmentPositionError" in mass_source
    assert "final_heading_bias_yaw_delta > kMaxPlacementAlignmentYawError" in mass_source
    assert "placement_alignment_physical[0] - final_heading_bias_x" in mass_source
    assert "placement_alignment_physical[1] - final_heading_bias_y" in mass_source
    assert "placement_alignment_physical[2] - final_heading_bias_yaw" in mass_source
    assert "navigation to final dispatch heading failed" in mass_source
    final_heading_navigation = mass_source.index(
        "node->navigate_to(final_heading_target, 120s)")
    assert alignment_segment_navigation < final_heading_navigation
    assert "final dispatch heading moved beyond the alignment segment bound" in mass_source
    assert "geometry_msgs::msg::Pose pre_place = grasp" in mass_source
    assert "kPrePlaceRadialClearance = 0.080" in mass_source
    assert "release_radial_yaw" in mass_source
    assert "above_release.position.z = pre_place.position.z" in mass_source
    assert "release -> above_release" in mass_source
    assert "above_release -> pre_place" in mass_source
    assert "retained_placement_solutions" in mass_source
    assert "product_attached" in mass_source[before_alignment_guard:alignment_navigation]
    assert 'native_attachment_state_is("attached")' in mass_source[
        before_alignment_guard:alignment_navigation]
    assert "top_down_radial_quaternion" in mass_source
    assert "pre_place.position.x = pre_place_base[0] +" in mass_source
    assert "pre_place.position.y = pre_place_base[1] +" in mass_source
    assert "kPrePlaceRadialClearance * release_base[0] / release_radius" in mass_source
    assert "kPrePlaceRadialClearance * release_base[1] / release_radius" in mass_source
    assert "const double map_aligned_product_yaw = wrap_yaw(-alignment_yaw)" in mass_source
    assert "map_aligned_product_yaw_pi = wrap_yaw" in mass_source
    assert "std::abs(map_aligned_product_yaw) <= std::abs(map_aligned_product_yaw_pi)" in mass_source
    assert "pre_place_map_yaw = wrap_yaw(pre_place_map_yaw + kProduct102PlacementYawOffset)" in mass_source
    assert "pre_place_map_yaw" in mass_source
    alignment_yaw = mass_source.index("const double alignment_yaw =")
    map_aligned_yaw = mass_source.index(
        "const double map_aligned_product_yaw =", alignment_yaw)
    pre_place_orientation = mass_source.index(
        "pre_place.orientation = amr_manipulation::top_down_radial_quaternion")
    assert alignment_yaw < map_aligned_yaw < pre_place_orientation
    orientation_start = mass_source.index(
        "geometry_msgs::msg::Quaternion top_down_radial_quaternion")
    orientation_end = mass_source.index(
        "}  // namespace amr_manipulation", orientation_start)
    orientation_source = mass_source[orientation_start:orientation_end]
    assert "q_z(radial_yaw) * q_y(+pi/2)" in orientation_source
    assert "held-product transform is q_y(-pi/2)" in orientation_source
    assert "result.x = -kQuarterTurn * std::sin(half_yaw)" in orientation_source
    assert "result.y = kQuarterTurn * std::cos(half_yaw)" in orientation_source
    assert "const double norm =" in orientation_source
    assert "result.x /= norm" in orientation_source
    assert "result.y /= norm" in orientation_source
    assert "result.z /= norm" in orientation_source
    assert "result.w /= norm" in orientation_source
    release_seed = mass_source.index("placement_release_seed{")
    release_preflight = mass_source.index("solve_retained_ik(release")
    pre_place_preflight = mass_source.index(
        "const auto & pre_place_ik_solution = retained_placement_solutions.back()")
    pre_place_ik = mass_source.index("arm.setJointValueTarget(pre_place_ik_solution)")
    pre_place_ompl = mass_source.index("arm.plan(pre_place_plan)")
    lower = mass_source.index("Measured pre-place endpoint error")
    placement_gate = mass_source.index("selected_slot_position_error() > 0.030", lower)
    continuation = mass_source.index("release_to_above_release_steps")
    detach_scene_remove = mass_source.index(
        "remove_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE")
    post_detach_evidence = mass_source.index(
        "fresh post-detach product scene evidence was unavailable", detach_scene_remove)
    finger_allow = mass_source.index(
        "set_held_product_finger_collision(true)", post_detach_evidence)
    post_retreat_waypoints = mass_source.index(
        "std::vector<geometry_msgs::msg::Pose> retreat_waypoints{pre_place}",
        finger_allow)
    post_retreat_path = mass_source.index(
        "retreat_waypoints, 0.005, 0.0, retreat_trajectory, true",
        post_retreat_waypoints)
    post_retreat_execute = mass_source.index(
        "arm.execute(retreat_plan)", post_retreat_path)
    restore_in_catch = mass_source.index(
        "set_held_product_finger_collision(false)", post_retreat_execute)
    restore_after_retreat = mass_source.index(
        "set_held_product_finger_collision(false)", restore_in_catch + 1)
    post_retreat_state = mass_source.index(
        "post_retreat_state = arm.getCurrentState(3.0)", restore_after_retreat)
    post_retreat_scene_proof = mass_source.index(
        "planning_scene_attached_object_proof(false)", post_retreat_state)
    post_retreat_response = mass_source.index(
        "validate_state(*post_retreat_state", post_retreat_scene_proof)
    post_retreat_gate = post_retreat_response
    empty_stow = mass_source.index(
        "arm.setJointValueTarget(stow)", post_retreat_gate)
    assert after_alignment_pose < release_seed < release_preflight
    assert release_preflight < continuation < pre_place_preflight
    lower_validation = mass_source.index(
        "Validate the measured-current-to-first-point segment", lower)
    lower_time_parameterization = mass_source.index(
        "IterativeParabolicTimeParameterization", lower_validation)
    lower_execution = mass_source.index("arm.execute(lower_plan)", lower_time_parameterization)
    assert pre_place_preflight < pre_place_ik < pre_place_ompl < lower < lower_validation
    assert lower_validation < lower_time_parameterization < lower_execution < placement_gate
    assert lower < detach_scene_remove < post_detach_evidence < finger_allow
    assert finger_allow < post_retreat_waypoints < post_retreat_path < post_retreat_execute
    assert post_retreat_execute < restore_in_catch < restore_after_retreat
    assert restore_after_retreat < post_retreat_state < post_retreat_scene_proof
    assert post_retreat_scene_proof < post_retreat_response < empty_stow
    assert "placement_ik_state->satisfiesBounds(manipulator_group)" in mass_source
    assert "expected_placement_joint_names" in mass_source
    assert '"arm_joint_1", "arm_joint_2", "arm_joint_3"' in mass_source
    assert "manipulator_group->getVariableNames() != expected_placement_joint_names" in mass_source
    assert "finite_joint_values" in mass_source
    assert "copyJointGroupPositions(manipulator_group, seed)" in mass_source
    assert "const auto & pre_place_ik_solution = retained_placement_solutions.back()" in mass_source
    assert "retained_placement_solutions" in mass_source
    assert "executed pre-place endpoint did not match retained IK branch" in mass_source
    assert '"/check_state_validity service was unavailable"' in mass_source
    assert "payload-aware state validity failed" in mass_source
    assert "lower_robot_trajectory" in mass_source
    assert "computeTimeStamps" in mass_source
    assert "lower_points.back().positions != release_ik_solution" not in mass_source
    assert "setJointGroupPositions(manipulator_group, seed)" in mass_source
    assert "std::ceil(distance / 0.005)" in mass_source
    assert "world_tcp_orientation" in mass_source
    assert "product_relative_orientation.y = -std::sqrt(0.5)" in mass_source
    assert "tcp_offset_world" in mass_source
    assert "orientation_dot" in mass_source
    assert "setApproximateJointValueTarget" not in mass_source
    assert "staging.orientation.y = std::sqrt(0.5)" in mass_source
    assert "attached.object.primitive_poses.front().orientation.y = -std::sqrt(0.5)" in mass_source
    assert "pre_place.orientation = amr_manipulation::top_down_radial_quaternion(\n      pre_place_map_yaw)" in mass_source
    assert "top_down_radial_quaternion(\n      pre_place_radial_yaw)" not in mass_source
    assert "placement_release_seed{\n      -release_radial_yaw" in mass_source
    assert "geometry_msgs::msg::Pose release = pre_place" in mass_source
    assert "placed_product" not in mass_source
    assert "set_held_product_finger_collision(true)" in mass_source
    assert mass_source.count("set_held_product_finger_collision(false)") == 2
    assert "gripper_left_finger_link" in mass_source
    assert "gripper_right_finger_link" in mass_source
    assert "ALLOWED_COLLISION_MATRIX" in mass_source
    assert 'request->group_name = "manipulator"' in mass_source
    assert "response->contacts" in mass_source
    assert "payload-aware state validity failed" in mass_source
    assert "request->robot_state.is_diff = true" in mass_source
    assert "planning_scene_attached_object_proof(true)" in mass_source
    assert "placement lower trajectory postconditions: PASS" in mass_source


def test_dispatch_stance_is_slot_aware_and_keeps_center_alignment_bounded():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    stance_start = source.index("const double selected_slot_lateral_offset")
    stance_end = source.index("const double dispatch_yaw", stance_start)
    stance = source[stance_start:stance_end]
    assert "selected_slot[1] - product.dispatch_dock[1]" in stance
    assert "const bool product102_center_slot =" in stance
    assert "product.id == 102 && selected_slot_lateral_offset == 0.0" in stance
    product102_start = stance.index("if (product102_center_slot)")
    upper_start = stance.index("} else if (selected_slot_lateral_offset > 0.0)", product102_start)
    product102_branch = stance[product102_start:upper_start]
    assert "desired_slot_direction_x = kDesiredProduct102SlotBaseX;" in product102_branch
    assert "desired_slot_direction_y = kDesiredProduct102SlotBaseY;" in product102_branch
    lower_start = stance.index("} else if (selected_slot_lateral_offset < 0.0)", upper_start)
    center_start = stance.index("} else {", lower_start)
    upper_branch = stance[upper_start:lower_start]
    lower_branch = stance[lower_start:center_start]
    center_branch = stance[center_start:]
    assert "desired_slot_direction_x = kDesiredSlotBaseX;" in upper_branch
    assert "desired_slot_direction_y = kDesiredUpperSlotBaseY;" in upper_branch
    assert "constexpr double kDesiredUpperSlotBaseY = -0.640000000;" in source
    assert "desired_slot_direction_x = kDesiredSlotBaseX;" in lower_branch
    assert "desired_slot_direction_y = -kDesiredSlotBaseY;" in lower_branch
    assert "desired_slot_direction_x = 1.0;" in center_branch
    assert "desired_slot_direction_y = 0.0;" in center_branch
    assert "const double desired_slot_base_radius = product102_center_slot ?" in stance
    assert "const double desired_slot_base_x = desired_slot_direction_x * desired_slot_scale;" in stance
    assert "const double desired_slot_base_y = desired_slot_direction_y * desired_slot_scale;" in stance
    assert "const double desired_slot_base_x = kDesiredSlotBaseX * desired_slot_scale;" not in stance
    assert "const double desired_slot_base_y = kDesiredSlotBaseY * desired_slot_scale;" not in stance
    assert "kDesiredProduct102SlotBaseX = 0.775000000" in source
    assert "kDesiredProduct102SlotBaseY = 0.075000000" in source
    assert "kProduct102PlacementLeadMapY = 0.085000000" in source
    assert "kProduct102PrePlaceZOffset = 0.100000000" in source
    assert "kProduct102PlacementYawOffset = 1.530000000" in source
    assert "placement_alignment_target_physical" in source

    desired_radius = 0.785 - 0.070 - 0.005
    center_stance = (-4.10 + desired_radius, 0.0)
    observed_dock = (-3.332, 0.002)
    center_alignment = math.hypot(
        center_stance[0] - observed_dock[0], center_stance[1] - observed_dock[1])
    assert math.isclose(center_stance[0], -3.390, abs_tol=1e-9)
    assert math.isclose(center_stance[1], 0.0, abs_tol=1e-9)
    assert math.isclose(center_alignment, 0.058034, abs_tol=0.0005)
    assert center_alignment < 0.35

    product102_radius = math.hypot(0.775, 0.075)
    product102_stance = (-4.10 + 0.775, 0.075)
    product102_alignment = math.hypot(
        product102_stance[0] - observed_dock[0],
        product102_stance[1] - observed_dock[1])
    product102_lead_target = (product102_stance[0], product102_stance[1] + 0.085)
    product102_lead_alignment = math.hypot(
        product102_lead_target[0] - observed_dock[0],
        product102_lead_target[1] - observed_dock[1])
    assert product102_radius < 0.785
    assert math.isclose(product102_radius, 0.778620575, abs_tol=1e-9)
    assert math.isclose(product102_alignment, 0.073334, abs_tol=0.0005)
    assert math.isclose(product102_lead_alignment, 0.158155, abs_tol=0.0005)
    assert product102_lead_alignment < 0.35


def test_product102_placement_branch_preserves_other_product_seeds_and_waypoints():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    assert "kProduct102PrePlaceZOffset = 0.100000000" in source
    assert "const double product_pre_place_z_offset = product102_center_slot ?" in source
    assert "std::vector<double> placement_release_seed{" in source
    assert (
        "-release_radial_yaw, 0.546225552, 0.335934775, 0.0, -0.882160326, 0.0"
    ) in source
    seed_start = source.index("std::vector<double> placement_release_seed{")
    seed_end = source.index("auto placement_ik_state", seed_start)
    product102_seed = source[seed_start:seed_end]
    assert "if (product102_center_slot)" in product102_seed
    assert "-1.693092000, 1.465170000, 2.432600000" in product102_seed
    assert "product.id == 103" not in product102_seed


def test_higher_mass_placement_uses_collision_aware_route_without_changing_1kg_path():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    execution = source.index("std::vector<std::vector<double>> execution_solutions;")
    one_kg_branch = source.index("if (product.id == 101)", execution)
    higher_mass_branch = source.index("} else {", one_kg_branch)
    common_validation = source.index(
        "const auto validate_interpolated_segment", higher_mass_branch)
    one_kg_source = source[one_kg_branch:higher_mass_branch]
    higher_mass_source = source[higher_mass_branch:common_validation]

    assert "retained_placement_solutions.rbegin()" in one_kg_source
    assert "arm.plan(collision_aware_lower_plan)" not in one_kg_source
    assert "arm.setStartStateToCurrentState()" in higher_mass_source
    assert "arm.setJointValueTarget(release_ik_solution)" in higher_mass_source
    assert "arm.plan(collision_aware_lower_plan)" in higher_mass_source
    assert "planned_trajectory.joint_names != expected_placement_joint_names" in higher_mass_source
    assert "planned_endpoint_error > 0.01" in higher_mass_source
    assert "execution_solutions.back() = release_ik_solution" in higher_mass_source
    assert "validate_interpolated_segment" not in higher_mass_source
    assert "planning_scene_attached_object_proof(true)" in source
    assert "placement lower trajectory postconditions: PASS" in source


def test_navigation_feedback_and_cancellation_contract_is_fail_closed():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    start = source.index("void reset_navigation_feedback")
    navigate_start = source.index("bool navigate_to(")
    end = source.index("bool dock_egress(", start)
    navigate = source[start:end]
    assert "reset_navigation_feedback()" in navigate
    assert "SendGoalOptions options" in navigate
    assert "options.feedback_callback" in navigate
    assert "record_navigation_feedback(*feedback)" in navigate
    assert "current_pose" in navigate
    assert "navigation_time" in navigate
    assert "distance_remaining" in navigate
    assert "result.wait_for(50ms)" in navigate
    assert "no navigation feedback within 5 wall seconds" in navigate
    assert "navigation feedback became stale" in navigate
    assert "navigation time became non-monotonic" in navigate
    assert "simulation navigation time exceeded the limit" in navigate
    assert "async_cancel_goal(goal_handle)" in navigate
    assert "response->goals_canceling" in navigate
    assert "goal_info.goal_id.uuid == goal_id" in navigate
    assert "terminal.code != rclcpp_action::ResultCode::CANCELED" in navigate
    assert "log_navigation_target(target)" in navigate
    assert "log_navigation_terminal(target, wrapped.code)" in navigate
    assert "result.wait_for(timeout)" not in source[navigate_start:end]


def test_gate6_pickup_approach_reverse_is_registry_bounded():
    launch_source = (ROOT / "launch" / "gate6_mass_stage.launch.py").read_text()
    assert "pickup_approach_distance = math.hypot" in launch_source
    assert "pickup_approach_distance <= 0.0" in launch_source
    assert "pickup_approach_distance > egress_max_distance" in launch_source
    assert "registered pickup approach reverse exceeds the arbitration distance limit" in launch_source


def test_mass_stage_publishes_motion_block_after_stationary_feedback():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    start = source.index("bool wait_for_motion_permission")
    end = source.index("bool capture_reference_evidence", start)
    permission = source[start:end]
    first_stationary_wait = permission.index("if (!wait_until_stationary")
    status_transition = permission.index("set_status(")
    announced = permission.index("const auto announced", status_transition)
    second_stationary_wait = permission.index("return wait_until_stationary", announced)
    assert first_stationary_wait < status_transition < announced < second_stationary_wait
    assert "now - stationary_since >= 500ms" in permission
    assert "now - announced >= 400ms" in permission
    assert "Dispatch detachment confirmed; opening gripper" not in source


def test_pickup_geometry_uses_fresh_relative_lateral_pose():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    reference = source.index("capture_reference_evidence(3s)")
    pickup_product_pose = source.index(
        "geometry_msgs::msg::PoseStamped pickup_product_pose", reference)
    pickup_robot_pose = source.index(
        "geometry_msgs::msg::PoseStamped pickup_robot_pose", pickup_product_pose)
    pickup_product_base = source.index(
        "const auto pickup_product_base = amr_manipulation::map_point_to_base",
        pickup_robot_pose)
    pickup_pedestal_base = source.index(
        "const auto pickup_pedestal_base = amr_manipulation::map_point_to_base",
        pickup_product_base)
    pickup_geometry_finite = source.index(
        '"pickup base-frame geometry was non-finite"', pickup_pedestal_base)
    pickup_scene = source.index("scene.applyCollisionObjects(obstacles)", pickup_geometry_finite)
    pickup_target = source.index(
        "pregrasp.position.y = pickup_product_lateral", pickup_scene)
    pregrasp_execute = source.index("arm.execute(pregrasp_plan)", pickup_target)

    assert reference < pickup_product_pose < pickup_robot_pose < pickup_product_base
    assert pickup_product_base < pickup_pedestal_base < pickup_geometry_finite < pickup_scene
    assert pickup_scene < pickup_target < pregrasp_execute
    assert "latest_product_pose(pickup_product_pose)" in source
    assert "latest_robot_pose(pickup_robot_pose)" in source
    assert "fresh pickup product or robot pose was unavailable" in source
    assert "fresh pickup product or robot pose was non-finite" in source
    assert "pickup_product_pose.pose.position.x + 0.05" in source
    assert "pickup_product_pose.pose.position.z - 0.45" in source
    assert "const double pickup_product_lateral = pickup_product_base[1]" in source
    assert "const double pickup_pedestal_lateral = pickup_pedestal_base[1]" in source
    assert '{0.85, pickup_product_lateral, 0.925}' in source
    assert '{0.90, pickup_pedestal_lateral, 0.375}' in source
    assert '{0.85, pickup_product_lateral, 0.825}' in source
    assert "wait_for_bilateral_contact(3s)" in source
    assert 'native_attachment_state_is("attached")' in source
