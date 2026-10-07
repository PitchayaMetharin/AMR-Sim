import importlib.util
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
MASS_STAGE_LAUNCH = ROOT / "launch" / "gate6_mass_stage.launch.py"
STANCE_HEADER = ROOT.parent / "amr_interfaces" / "include" / "amr_interfaces" / "final_placement_stance.hpp"


def _assert_fail_closed_block(helper, condition, next_stage):
    normalized = " ".join(helper.split())
    condition = " ".join(condition.split())
    next_stage = " ".join(next_stage.split())
    start = normalized.index(condition)
    end = normalized.index(next_stage, start + len(condition))
    block = normalized[start:end].strip()
    pattern = (
        re.escape(condition)
        + r"\s*\{\s*RCLCPP_ERROR\([^;{}]*\);"
        + r"\s*return false;\s*\}"
    )
    assert re.fullmatch(pattern, block), condition


def _assert_guarded_throw_block(source, condition, message):
    normalized = " ".join(source.split())
    condition = " ".join(condition.split())
    throw_statement = f'throw std::runtime_error("{message}");'
    start = normalized.index(condition)
    throw_start = normalized.index(throw_statement, start)
    end = throw_start + len(throw_statement)
    block = normalized[start:end].strip()
    pattern = (
        re.escape(condition)
        + r"\s*(?:\{\s*)?"
        + re.escape(throw_statement)
        + r"\s*(?:\})?"
    )
    assert re.fullmatch(pattern, block), condition


def _main_placement_alignment_loop(source):
    alignment_count = source.index("std::size_t alignment_segment_count = 0;")
    loop_start = source.index("while (", alignment_count)
    loop_end = source.index("// Finish the bounded translation", loop_start)
    return source[loop_start:loop_end]


def _load_mass_stage_launch():
    spec = importlib.util.spec_from_file_location(
        "gate6_mass_stage_launch_contract", MASS_STAGE_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mass_stage_launch_propagates_nested_exit_code():
    launch = _load_mass_stage_launch()
    callback = launch._mass_stage_exit

    assert callback(SimpleNamespace(returncode=0), None) == []
    for returncode in (None, 127, 130, 23):
        with pytest.raises(RuntimeError, match=rf"status {returncode}"):
            callback(SimpleNamespace(returncode=returncode), None)


def test_mass_stage_launch_registers_exit_handler_before_stage_node(monkeypatch):
    launch = _load_mass_stage_launch()
    monkeypatch.setattr(launch, "_resolve_registry", lambda _context: {})
    moveit_config = SimpleNamespace(to_dict=lambda: {})

    actions = launch._make_node(SimpleNamespace(), moveit_config)

    assert isinstance(actions[0], launch.RegisterEventHandler)
    assert isinstance(actions[1], launch.Node)


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
        "pregrasp.position.x = pickup_product_base[0];\n"
        "    pregrasp.position.y = pickup_product_lateral;\n"
        "    pregrasp.position.z = 1.00;"
    ) in mass_source
    assert 'arm.setJointValueTarget(pregrasp, "gripper_tcp")' not in mass_source
    assert "const double tcp_x_world = std::cos(robot_yaw) * tcp.pose.position.x" in mass_source
    assert "const double tcp_y_world = std::sin(robot_yaw) * tcp.pose.position.x" in mass_source
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
    pregrasp_constraints = mass_source.index(
        "moveit_msgs::msg::Constraints pregrasp_wrist_constraints", pregrasp_target)
    pregrasp_path_constraint = mass_source.index(
        "arm.setPathConstraints(pregrasp_wrist_constraints)", pregrasp_constraints)
    pregrasp_ompl = mass_source.index("arm.plan(pregrasp_plan)")
    pregrasp_clear_constraints = mass_source.index(
        "arm.clearPathConstraints()", pregrasp_ompl)
    assert pregrasp_seed < pregrasp_state < pregrasp_group
    assert pregrasp_group < pregrasp_order < pregrasp_count
    assert pregrasp_count < pregrasp_finite_seed < pregrasp_set_seed
    assert pregrasp_set_seed < pregrasp_ik < pregrasp_bounds < pregrasp_copy
    assert pregrasp_copy < pregrasp_finite_solution < pregrasp_start
    assert pregrasp_start < pregrasp_target < pregrasp_constraints
    assert pregrasp_constraints < pregrasp_path_constraint < pregrasp_ompl
    assert pregrasp_ompl < pregrasp_clear_constraints
    assert 'unsafe wrist-flipped pre-grasp branch rejected' in mass_source
    assert "command_gripper(node, 0.020)" in mass_source
    command_start = mass_source.index("bool command_gripper")
    command_end = mass_source.index(
        "std::array<double, 3> map_point_to_base", command_start)
    command_source = mass_source[command_start:command_end]
    assert '"/gripper_controller/gripper_cmd"' in command_source
    assert '"/gripper_right_controller/gripper_cmd"' in command_source
    left_send = command_source.index(
        "left_client->async_send_goal(left_goal, make_options(left_client, left_pending, \"left\"));")
    right_send = command_source.index(
        "right_client->async_send_goal(right_goal, make_options(right_client, right_pending, \"right\"));")
    acceptance_wait = command_source.index(
        "const auto acceptance_deadline = std::chrono::steady_clock::now() + 3s", right_send)
    assert left_send < right_send < acceptance_wait
    assert command_source.count("async_send_goal") == 2
    assert "PendingActionGoal<Action>" in command_source
    assert "pending->condition.notify_all()" in command_source
    assert "pending->abandoned" in command_source
    assert "async_cancel_goal(goal_handle)" in command_source
    assert "std::this_thread::sleep_for(20ms)" in command_source
    assert "std::shared_ptr<GoalHandle> left_goal_handle;" in command_source
    assert "std::shared_ptr<GoalHandle> right_goal_handle;" in command_source
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
    result_deadline = command_source.index(
        "const auto deadline = std::chrono::steady_clock::now() + 30s")
    result_poll = command_source.index("result.wait_for(50ms)", result_deadline)
    result_timeout = command_source.index(
        "result.wait_for(0s) != std::future_status::ready", result_poll)
    timeout_cancel = command_source.index(
        "cancel_accepted_goal(side, client, goal_handle, result)", result_timeout)
    timeout_return = command_source.index("return false;", timeout_cancel)
    assert result_deadline < result_poll < result_timeout < timeout_cancel < timeout_return
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
    assert '"/amr/mission/navigate_to_pose_precise"' in mass_source
    assert "precise_navigation_client_" in mass_source
    assert mass_source.count("node->navigate_to_aligned_precision(") == 2
    assert mass_source.count("node->navigate_product102_clear_approach(") == 0
    assert mass_source.count("node->navigate_product102_centered_dock(") == 1
    centered_method_start = mass_source.index("bool navigate_product102_centered_dock(")
    centered_method_end = mass_source.index("bool bounded_reverse(", centered_method_start)
    centered_method = mass_source[centered_method_start:centered_method_end]
    assert "factory_world" in centered_method
    assert "latest_current_tf_pose" in centered_method
    assert "dispatch_b_clear_approach_client_" in centered_method
    assert "wait_stopped" in centered_method and "wall_started + 8s" in centered_method
    assert "wall_now - stationary_since >= 500ms" in centered_method
    assert "physical_stance_xy_outside_tolerance" in centered_method
    assert "bool precise" not in mass_source
    assert "precise ?" not in mass_source
    assert "navigation_client_" in mass_source
    assert 'geometry_msgs/msg/pose_with_covariance_stamped.hpp' in mass_source
    assert '"/amr/amcl_pose"' in mass_source
    navigation_start = mass_source.index("bool navigate_to_with_client(")
    navigation_end = mass_source.index("bool navigate_to(", navigation_start)
    navigation_source = mass_source[navigation_start:navigation_end]
    assert "use_fresh_amcl_terminal_pose" in navigation_source
    assert "wrapped.code == rclcpp_action::ResultCode::SUCCEEDED" in navigation_source
    assert "no_navigation_feedback" in navigation_source
    assert "using fresh AMCL terminal pose" in mass_source
    assert "position_error > 0.07 || yaw_error > 0.15" in mass_source
    assert 'ensure_entry("held_product")' in mass_source
    assert 'ensure_entry("pickup_pedestal")' in mass_source
    negative_check = mass_source.index("Out-of-dispatch detachment rejection")
    egress_call = mass_source.index("node->dock_egress(65s)")
    pickup_reverse_geometry = mass_source.index(
        "dock_to_egress_dx", egress_call)
    pickup_reverse_distance = mass_source.index(
        "pickup_approach_distance = std::hypot", pickup_reverse_geometry)
    pickup_retreat = mass_source.index(
        "if (!node->navigate_to_registered_retreat(product.pickup_station, 120s))",
        pickup_reverse_distance)
    pickup_heading = mass_source.index(
        "if (!node->navigate_to(product.pickup_station, 120s))", pickup_retreat)
    assert negative_check < egress_call < pickup_reverse_geometry
    assert pickup_reverse_geometry < pickup_reverse_distance < pickup_retreat < pickup_heading

    pickup_caller_end = mass_source.index(
        "geometry_msgs::msg::PoseStamped dispatch_dock_bias_ground_truth",
        pickup_retreat)
    pickup_caller = mass_source[pickup_retreat:pickup_caller_end]
    caller_retreat = pickup_caller.index(
        "node->navigate_to_registered_retreat(product.pickup_station, 120s)")
    caller_heading = pickup_caller.index(
        "node->navigate_to(product.pickup_station, 120s)", caller_retreat)
    caller_admission = pickup_caller.index(
        "if (!amr_manipulation::pickup_station_admission_proof(", caller_heading)
    caller_failure = pickup_caller.index(
        'throw std::runtime_error("pickup station admission proof failed")',
        caller_admission)
    caller_translation_heading = pickup_caller.index(
        "const double dispatch_translation_heading = std::atan2(", caller_failure)
    caller_navigation = pickup_caller.index(
        "node->navigate_to_dispatch(dispatch_translation_target, 120s)", caller_heading)
    assert caller_retreat < caller_heading < caller_admission < caller_failure
    assert caller_failure < caller_translation_heading < caller_navigation
    assert "node, product, product_attached, pickup_station_achieved, dispatch_translation_start" in pickup_caller[caller_admission:caller_failure]
    _assert_guarded_throw_block(
        pickup_caller,
        "if (!node->navigate_to_registered_retreat(product.pickup_station, 120s))",
        "pickup station retreat failed",
    )
    _assert_guarded_throw_block(
        pickup_caller,
        "if (!node->navigate_to(product.pickup_station, 120s))",
        "pickup station heading alignment failed",
    )
    _assert_guarded_throw_block(
        pickup_caller,
        (
            "if (!amr_manipulation::pickup_station_admission_proof(\n"
            "    node, product, product_attached, pickup_station_achieved,\n"
            "    dispatch_translation_start))"
        ),
        "pickup station admission proof failed",
    )

    helper_start = mass_source.index(
        "bool pickup_station_admission_proof(", mass_source.index("int main("))
    helper_end = mass_source.index(
        "\n}  // namespace amr_manipulation", helper_start)
    helper = mass_source[helper_start:helper_end]
    helper_attachment = helper.index(
        'if (!product_attached || !node->native_attachment_state_is("attached"))')
    helper_reset = helper.index("node->reset_navigation_feedback()", helper_attachment)
    helper_amcl = helper.index(
        "node->wait_for_amcl_terminal_pose(product.pickup_station)", helper_reset)
    helper_achieved = helper.index(
        "latest_navigation_feedback_pose(pickup_station_achieved)", helper_amcl)
    helper_xy = helper.index(
        "const double pickup_station_xy_error = std::hypot(", helper_achieved)
    helper_yaw = helper.index(
        "const double pickup_station_yaw = std::atan2(", helper_xy)
    helper_yaw_error = helper.index(
        "const double pickup_station_yaw_error = std::abs", helper_yaw)
    helper_finite = helper.index(
        "!std::isfinite(pickup_station_xy_error)", helper_yaw_error)
    helper_gate = helper.index(
        "pickup_station_xy_error > 0.07 || pickup_station_yaw_error > 0.15",
        helper_finite)
    helper_dispatch = helper.index(
        "dispatch_translation_start = pickup_station_achieved", helper_gate)
    helper_success = helper.index("return true;", helper_dispatch)
    assert helper_attachment < helper_reset < helper_amcl < helper_achieved
    assert helper_achieved < helper_xy < helper_yaw < helper_yaw_error < helper_finite
    assert helper_finite < helper_gate < helper_dispatch < helper_success
    assert "!std::isfinite(pickup_station_yaw_error)" in helper[helper_finite:helper_gate]
    for condition, next_stage in (
        (
            'if (!product_attached || !node->native_attachment_state_is("attached"))',
            "node->reset_navigation_feedback();",
        ),
        (
            "if (!node->wait_for_amcl_terminal_pose(product.pickup_station))",
            "if (!node->latest_navigation_feedback_pose(pickup_station_achieved))",
        ),
        (
            "if (!node->latest_navigation_feedback_pose(pickup_station_achieved))",
            "const double pickup_station_xy_error",
        ),
        (
            "if (!std::isfinite(pickup_station_xy_error) || "
            "!std::isfinite(pickup_station_yaw_error) || "
            "pickup_station_xy_error > 0.07 || pickup_station_yaw_error > 0.15)",
            "dispatch_translation_start = pickup_station_achieved;",
        ),
    ):
        _assert_fail_closed_block(helper, condition, next_stage)

    dispatch_translation_start = mass_source.index(
        "const double dispatch_translation_heading = std::atan2(",
        pickup_retreat)
    dispatch_translation_bearing = dispatch_translation_start
    dispatch_translation_navigation = mass_source.index(
        "node->navigate_to_dispatch(dispatch_translation_target, 120s)",
        dispatch_translation_bearing)
    translation_attachment = mass_source.index(
        "attachment proof failed after dispatch approach translation",
        dispatch_translation_navigation)
    dispatch_heading_start = mass_source.index(
        "dispatch_heading_start", translation_attachment)
    dispatch_heading_navigation = mass_source.index(
        "node->navigate_to_dispatch(dispatch_heading_target, 120s)", dispatch_heading_start)
    heading_attachment = mass_source.index(
        "attachment proof failed after dispatch approach heading",
        dispatch_heading_navigation)
    assert pickup_retreat < dispatch_translation_start < dispatch_translation_navigation
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
    egress_deadline = egress_source.index(
        "const auto deadline = std::chrono::steady_clock::now() + client_timeout")
    egress_poll = egress_source.index("result.wait_for(50ms)", egress_deadline)
    egress_timeout = egress_source.index(
        "result.wait_for(0s) != std::future_status::ready", egress_poll)
    assert "async_cancel_goal(goal_handle)" in egress_source
    assert "response->goals_canceling" in egress_source
    assert "terminal.code != rclcpp_action::ResultCode::CANCELED" in egress_source
    assert egress_deadline < egress_poll < egress_timeout
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
        "node->navigate_to_aligned_precision(dispatch_dock_corrected_target, 120s, true)", dock_target)
    dock_attachment = mass_source.index(
        "attachment proof failed after dispatch dock", dock_navigation)
    registered_dock_check = mass_source.index(
        "node->dock_pose_within_tolerance(5s)", dock_attachment)
    alignment_goal = mass_source.index("placement_alignment{")
    alignment_segment_navigation = mass_source.index(
        "node->navigate_to_aligned_precision(segment_target, 120s, true)")
    alignment_navigation = mass_source.index(
        "node->navigate_to_aligned_precision(segment_target, 120s, true)")
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
    assert helper_gate < helper_dispatch
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
    assert "node->wait_for_amcl_terminal_pose(product.pickup_station)" in mass_source
    assert "if (use_fresh_amcl_terminal_pose(target))" in mass_source
    assert "node->navigate_to(product.pickup_station, 120s)" in mass_source
    assert "node->navigate_to_registered_retreat(product.pickup_station, 120s)" in mass_source
    assert "navigate_to_with_client" in mass_source
    assert '"/amr/mission/navigate_to_pose_retreat"' in mass_source
    assert "retreat_navigation_client_" in mass_source
    assert "pickup_station_bearing_target" not in mass_source
    assert "pickup_station_heading_target" not in mass_source
    assert "pickup_travel_bearing" not in mass_source
    assert "pickup station retreat geometry was invalid" in mass_source
    assert "pickup_approach_distance > product.pickup_egress_max_distance_m" in mass_source
    assert "pickup_station_xy_error > 0.07" in mass_source
    assert "pickup_station_yaw_error > 0.15" in mass_source
    assert "node->navigate_to_dispatch(dispatch_translation_target, 120s)" in mass_source
    assert "node->navigate_to_dispatch(dispatch_heading_target, 120s)" in mass_source
    assert mass_source.count(
        "node->navigate_to_aligned_precision(dispatch_dock_corrected_target, 120s, true)") == 1
    assert "node->navigate_to(product.dispatch_dock, 120s)" not in mass_source
    assert "node->navigate_to(product.dispatch_dock, 120s, true)" not in mass_source
    assert re.search(
        r"const std::array<double, 3> dispatch_entry_physical = product102_center_slot \?\s*"
        r"final_placement_stance\.physical : product\.dispatch_dock;",
        mass_source,
    )
    assert "dispatch_entry_physical[0] - dispatch_dock_bias_x" in mass_source
    assert "dispatch_entry_physical[1] - dispatch_dock_bias_y" in mass_source
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
    stance_source = STANCE_HEADER.read_text()
    assert "std::cos(dispatch_yaw)" in stance_source
    assert "std::sin(dispatch_yaw)" in stance_source
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
    assert "kDesiredSlotBaseX = 0.520000000" in stance_source
    assert "kDesiredSlotBaseY = -0.580000000" in stance_source
    assert "kPlacementReachReserve = 0.005" in stance_source
    assert "kDesiredSlotBaseRadius =" in stance_source
    assert "kMaxPlacementReleaseRadius - kMaxPlacementAlignmentPositionError" in stance_source
    assert "desired_slot_direction_radius =" in stance_source
    assert "const double desired_slot_base_radius = product102_center_slot ?" in stance_source
    assert "desired_slot_scale = desired_slot_base_radius / desired_slot_direction_radius" in stance_source
    assert "desired_slot_base_x = desired_slot_direction_x * desired_slot_scale" in stance_source
    assert "desired_slot_base_y = desired_slot_direction_y * desired_slot_scale" in stance_source
    assert "desired placement stance radius was invalid" in mass_source
    assert "desired placement stance scaling was non-finite" in mass_source
    assert "placement_alignment_physical" in mass_source
    assert "placement_alignment" in mass_source
    assert "alignment_segments" in mass_source
    assert "alignment_segment_count < 8" in mass_source
    assert "latest_robot_pose(current_alignment_ground_truth)" in mass_source
    assert "latest_navigation_feedback_pose(current_alignment_localized)" in mass_source
    assert "step_distance = std::min" in mass_source
    assert "bounded dispatch placement alignment did not converge" in mass_source
    assert "const double forward_heading = std::atan2(remaining_dy, remaining_dx)" in mass_source
    assert "const std::array<double, 3> segment_target{" in mass_source
    assert "step_target_physical_y - current_bias_y,\n        wrap_yaw(segment_heading - current_bias_yaw)}" in mass_source
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
        "node->navigate_to_dispatch(final_heading_target, 120s)")
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


def test_product102_centered_entry_uses_stopped_current_tf_clear_and_dock():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    begin = source.index("bool navigate_product102_centered_dock(")
    end = source.index("bool bounded_reverse(", begin)
    route = source[begin:end]
    assert "product_.dispatch_approach[0]" in route
    assert "product_.dispatch_approach[1]" in route
    assert "const double projection" in route
    assert "clear_point" in route
    assert "dispatch_b_clear_approach_client_" in route
    assert "dispatch_precise_navigation_client_" in route
    assert "navigate_to_aligned_precision" not in route
    assert "latest_navigation_feedback_pose" not in route
    assert "kFinalHeadingGoalMargin" not in route
    assert "factory_world" in route
    assert "original_clear_reference" in route
    assert "kClearAreaDisplacement = 0.15" in route
    assert "kRegisteredApproachAdmission = 0.155" in route
    assert "physical_clear_error <= kClearPositionTolerance" in route
    assert "attempt != 0" in route

    main_start = source.index("const auto final_placement_stance")
    centered_route_start = source.index("if (product102_center_slot) {", main_start)
    centered_route_end = source.index("} else {", centered_route_start)
    centered_route = source[centered_route_start:centered_route_end]
    assert "navigate_product102_centered_dock(" in centered_route
    assert "final_placement_stance.physical" in centered_route
    assert "dispatch_dock_bias_ground_truth = centered_dock.physical" in centered_route
    assert "while (!centered_dock_only_admitted && alignment_segment_count < 8)" in " ".join(
        source.split())


def test_product102_centered_admission_skips_entire_main_translation_loop():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()

    admission_start = source.index(
        "CenteredBAlignmentAdmission centered_b_alignment_admission(")
    admission_end = source.index("\ntemplate<typename ActionT>", admission_start)
    admission = source[admission_start:admission_end]
    assert "std::isfinite(residual_m)" in admission
    assert "residual_m >= 0.0" in admission
    assert "residual_m <= 0.010" in admission
    assert "CenteredBAlignmentAdmission::DOCK_ONLY" in admission

    main_admission_start = source.index(
        "const bool centered_dock_only_admitted =")
    segments_start = source.index("const auto alignment_segments =", main_admission_start)
    main_admission = source[main_admission_start:segments_start]
    assert "product102_center_slot &&" in main_admission
    assert "centered_b_alignment_admission(alignment_displacement)" in main_admission
    assert "CenteredBAlignmentAdmission::DOCK_ONLY" in main_admission

    segments_end = source.index("const bool alignment_geometry_finite =", segments_start)
    alignment_segments = source[segments_start:segments_end]
    assert "centered_dock_only_admitted ? 0U :" in alignment_segments

    finite_gate = source.index("if (!alignment_geometry_finite ||", segments_end)
    attachment_gate = source.index(
        'if (!product_attached || !node->native_attachment_state_is("attached"))',
        finite_gate,
    )
    loop_start = source.index("while (", attachment_gate)
    loop_open = source.index("{", loop_start)
    loop_condition = " ".join(source[loop_start:loop_open].split())
    assert loop_condition == (
        "while (!centered_dock_only_admitted && alignment_segment_count < 8)"
    ), "admitted centered B must skip the entire short-translation loop"
    assert finite_gate < attachment_gate < loop_start
    assert "alignment_displacement > kMaxPlacementAlignmentTotalDisplacement" in source[
        finite_gate:attachment_gate]

    loop_end = source.index("// Finish the bounded translation", loop_start)
    convergence = source.index(
        "if (remaining_alignment_distance > placement_translation_position_tolerance)",
        loop_start,
        loop_end,
    )
    final_heading = source.index(
        "geometry_msgs::msg::PoseStamped final_heading_ground_truth;", loop_end)
    require_permission = source.index("require_motion_permission()", final_heading)
    assert loop_start < convergence < loop_end < final_heading < require_permission
    assert "alignment_segment_count < 8" in loop_condition


def test_stage_start_waits_for_real_public_boundary_before_motion_permission():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    bootstrap = source.index("if (!node->verify_attachment_bootstrap(5s))")
    forwarded = source.index("node->wait_for_stage_start_forwarded(8s)", bootstrap)
    permission = source.index("require_motion_permission();", forwarded)
    assert bootstrap < forwarded < permission
    assert "forwarded_status_received_ >= started" in source
    assert "now_wall - forwarded_status_received_ <= 200ms" in source
    assert "status.source_boot_id != boot_id_" in source
    assert "!status.base_motion_allowed" in source
    assert 'status.detail == "Gate 6 mass stage is starting"' in source


def test_dispatch_stance_is_slot_aware_and_keeps_center_alignment_bounded():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    stance_source = STANCE_HEADER.read_text()
    stance_start = stance_source.index("const double selected_slot_lateral_offset")
    stance_end = stance_source.index("const double dispatch_yaw", stance_start)
    stance = stance_source[stance_start:stance_end]
    assert "selected_slot[1] - dispatch_dock[1]" in stance
    assert "const bool product102_center_slot =" in stance
    assert "product_id == 102 && selected_slot_lateral_offset == 0.0" in stance
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
    assert "constexpr double kDesiredUpperSlotBaseY = -0.640000000;" in stance_source
    assert "desired_slot_direction_x = kDesiredSlotBaseX;" in lower_branch
    assert "desired_slot_direction_y = -kDesiredSlotBaseY;" in lower_branch
    assert "desired_slot_direction_x = 1.0;" in center_branch
    assert "desired_slot_direction_y = 0.0;" in center_branch
    assert "const double desired_slot_base_radius = product102_center_slot ?" in stance
    assert "const double desired_slot_base_x = desired_slot_direction_x * desired_slot_scale;" in stance
    assert "const double desired_slot_base_y = desired_slot_direction_y * desired_slot_scale;" in stance
    assert "const double desired_slot_base_x = kDesiredSlotBaseX * desired_slot_scale;" not in stance
    assert "const double desired_slot_base_y = kDesiredSlotBaseY * desired_slot_scale;" not in stance
    assert "kDesiredProduct102SlotBaseX = 0.748000000" in stance_source
    assert "kDesiredProduct102SlotBaseY = 0.100000000" in stance_source
    helper_call = source.index("const auto final_placement_stance = amr_interfaces::placement::final_placement_stance(")
    transport = source.index("node->navigate_to_dispatch(dispatch_translation_target, 120s)")
    physical = source.index("const auto placement_alignment_physical = final_placement_stance.physical;")
    assert helper_call < transport < physical
    assert "product.id, product.dispatch_dock,\n      product.dispatch_slots.at(product.selected_slot_index)" in source
    assert "const double desired_slot_direction_radius = final_placement_stance.direction_radius;" in source
    assert "const double desired_slot_base_radius = final_placement_stance.base_radius;" in source
    assert "const double desired_slot_scale = final_placement_stance.scale;" in source
    assert "const double desired_slot_base_x = final_placement_stance.base_x;" in source
    assert "const double desired_slot_base_y = final_placement_stance.base_y;" in source
    assert "kProduct102PlacementLeadMapY" not in source
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

    product102_radius = math.hypot(0.748, 0.100)
    product102_stance = (-4.10 + 0.748, 0.100)
    product102_alignment = math.hypot(
        product102_stance[0] - observed_dock[0],
        product102_stance[1] - observed_dock[1])
    assert product102_radius < 0.785
    assert math.isclose(product102_radius, 0.754654888, abs_tol=1e-9)
    assert math.isclose(product102_alignment, 0.100019998, abs_tol=1e-9)
    assert product102_alignment < 0.35
    # Run15 accepted this pose near the obsolete coarse-route lead. It is
    # inside the physical XY envelope but outside the unchanged reach gate.
    run15_base = (-3.320295424392525, 0.11835553132761523)
    run15_stance_error = math.hypot(
        run15_base[0] - product102_stance[0], run15_base[1] - product102_stance[1])
    run15_release_radius = math.hypot(-4.10 - run15_base[0], -run15_base[1])
    assert run15_stance_error < 0.07
    assert run15_release_radius > 0.785
    assert math.isclose(run15_release_radius, 0.7886363274786893, abs_tol=1e-12)


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
    navigate_start = source.index("bool navigate_to_with_client(")
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


def test_post_grasp_pickup_retreat_uses_registered_navigation_and_terminal_proof():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()

    assert '"/amr/mission/navigate_to_pose"' in source
    assert '"/amr/mission/navigate_to_pose_retreat"' in source
    assert "retreat_navigation_client_" in source
    assert "navigate_to_registered_retreat" in source
    assert "navigate_to_with_client" in source
    assert "node->navigate_to(product.pickup_station, 120s)" in source

    egress = source.index("node->dock_egress(65s)")
    retreat = source.index(
        "if (!node->navigate_to_registered_retreat(product.pickup_station, 120s))")
    caller_end = source.index(
        "geometry_msgs::msg::PoseStamped dispatch_dock_bias_ground_truth", retreat)
    caller = source[retreat:caller_end]
    caller_retreat = caller.index(
        "node->navigate_to_registered_retreat(product.pickup_station, 120s)")
    caller_heading = caller.index(
        "node->navigate_to(product.pickup_station, 120s)", caller_retreat)
    caller_admission = caller.index(
        "if (!amr_manipulation::pickup_station_admission_proof(", caller_heading)
    caller_failure = caller.index(
        'throw std::runtime_error("pickup station admission proof failed")',
        caller_admission)
    caller_translation_heading = caller.index(
        "const double dispatch_translation_heading = std::atan2(", caller_failure)
    caller_navigation = caller.index(
        "node->navigate_to_dispatch(dispatch_translation_target, 120s)", caller_heading)
    assert caller_retreat < caller_heading < caller_admission < caller_failure
    assert caller_failure < caller_translation_heading < caller_navigation
    assert "node, product, product_attached, pickup_station_achieved, dispatch_translation_start" in caller[caller_admission:caller_failure]
    _assert_guarded_throw_block(
        caller,
        "if (!node->navigate_to_registered_retreat(product.pickup_station, 120s))",
        "pickup station retreat failed",
    )
    _assert_guarded_throw_block(
        caller,
        "if (!node->navigate_to(product.pickup_station, 120s))",
        "pickup station heading alignment failed",
    )
    _assert_guarded_throw_block(
        caller,
        (
            "if (!amr_manipulation::pickup_station_admission_proof(\n"
            "    node, product, product_attached, pickup_station_achieved,\n"
            "    dispatch_translation_start))"
        ),
        "pickup station admission proof failed",
    )
    assert "product.dispatch_approach[0]" in caller[caller_heading:caller_navigation]
    assert "product.dispatch_approach[1]" in caller[caller_heading:caller_navigation]
    assert "std::atan2" in caller[caller_heading:caller_navigation]

    helper_start = source.index("bool pickup_station_admission_proof(", source.index("int main("))
    helper_end = source.index("\n}  // namespace amr_manipulation", helper_start)
    helper = source[helper_start:helper_end]
    helper_attachment = helper.index(
        'if (!product_attached || !node->native_attachment_state_is("attached"))')
    helper_reset = helper.index("node->reset_navigation_feedback()", helper_attachment)
    helper_amcl = helper.index(
        "node->wait_for_amcl_terminal_pose(product.pickup_station)", helper_reset)
    helper_achieved = helper.index(
        "latest_navigation_feedback_pose(pickup_station_achieved)", helper_amcl)
    helper_xy = helper.index(
        "const double pickup_station_xy_error = std::hypot(", helper_achieved)
    helper_yaw = helper.index(
        "const double pickup_station_yaw = std::atan2(", helper_xy)
    helper_yaw_error = helper.index(
        "const double pickup_station_yaw_error = std::abs", helper_yaw)
    helper_finite = helper.index(
        "!std::isfinite(pickup_station_xy_error)", helper_yaw_error)
    helper_gate = helper.index(
        "pickup_station_xy_error > 0.07 || pickup_station_yaw_error > 0.15",
        helper_finite)
    helper_dispatch = helper.index(
        "dispatch_translation_start = pickup_station_achieved", helper_gate)
    helper_success = helper.index("return true;", helper_dispatch)
    assert helper_attachment < helper_reset < helper_amcl < helper_achieved
    assert helper_achieved < helper_xy < helper_yaw < helper_yaw_error < helper_finite
    assert helper_finite < helper_gate < helper_dispatch < helper_success
    assert "!std::isfinite(pickup_station_yaw_error)" in helper[helper_finite:helper_gate]
    for condition, next_stage in (
        (
            'if (!product_attached || !node->native_attachment_state_is("attached"))',
            "node->reset_navigation_feedback();",
        ),
        (
            "if (!node->wait_for_amcl_terminal_pose(product.pickup_station))",
            "if (!node->latest_navigation_feedback_pose(pickup_station_achieved))",
        ),
        (
            "if (!node->latest_navigation_feedback_pose(pickup_station_achieved))",
            "const double pickup_station_xy_error",
        ),
        (
            "if (!std::isfinite(pickup_station_xy_error) || "
            "!std::isfinite(pickup_station_yaw_error) || "
            "pickup_station_xy_error > 0.07 || pickup_station_yaw_error > 0.15)",
            "dispatch_translation_start = pickup_station_achieved;",
        ),
    ):
        _assert_fail_closed_block(helper, condition, next_stage)
    assert egress < retreat
    assert "using fresh AMCL terminal pose within existing tolerance" in source
    assert "Navigation succeeded without feedback" not in source

    assert source.count("node->navigate_to_dispatch(dispatch_translation_target, 120s)") == 1
    assert source.count("node->navigate_to_dispatch(dispatch_heading_target, 120s)") == 1
    assert source.count("node->navigate_to_aligned_precision(dispatch_dock_corrected_target, 120s, true)") == 1
    assert source.count("node->navigate_to_aligned_precision(segment_target, 120s, true)") == 1
    assert source.count("node->navigate_to_dispatch(final_heading_target, 120s)") == 1

    navigation_start = source.index("bool navigate_to_with_client(")
    navigation_end = source.index("bool navigate_to(", navigation_start)
    navigation = source[navigation_start:navigation_end]
    assert "navigation_client->wait_for_action_server(5s)" in navigation
    assert "navigation_client->async_send_goal(goal, options)" in navigation
    assert "[this, pending, navigation_client]" in navigation
    assert "navigation_client->async_get_result(goal_handle)" in navigation
    entry_guard = navigation.index("if (guard && !guard())")
    server_wait = navigation.index("navigation_client->wait_for_action_server(5s)")
    guard_after_server_wait = navigation.index(
        "if (guard && !guard())", server_wait)
    send_goal = navigation.index("navigation_client->async_send_goal(goal, options)")
    guard_before_send = navigation.rindex("if (guard && !guard())", 0, send_goal)
    assert entry_guard < server_wait < guard_after_server_wait < guard_before_send < send_goal

    acceptance_deadline = navigation.index(
        "const auto acceptance_deadline = std::chrono::steady_clock::now() + 5s")
    acceptance_loop = navigation.index("while (rclcpp::ok()) {", acceptance_deadline)
    acceptance_guard = navigation.index("if (guard && !guard())", acceptance_loop)
    abandoned_pending = navigation.index("pending->abandoned = true;", acceptance_guard)
    acceptance_wait = navigation.index(
        "pending->condition.wait_for(pending_lock, 50ms)", abandoned_pending)
    assert send_goal < acceptance_loop < acceptance_guard < abandoned_pending < acceptance_wait

    assert navigation.count(
        "cancel_navigation_goal(navigation_client, goal_handle, result)") == 5
    assert navigation.count(
        "(void)cancel_navigation_goal(navigation_client, goal_handle, result)") == 4

    cancel_start = source.index("bool cancel_navigation_goal(")
    cancel_helper = source[cancel_start:navigation_start]
    assert "async_cancel_goal(goal_handle)" in cancel_helper
    assert "goal_info.goal_id.uuid == goal_id" in cancel_helper
    assert "terminal.code != rclcpp_action::ResultCode::CANCELED" in cancel_helper

    guarded_rejection = navigation.index("const auto reject_guarded_goal =")
    ready_branch = navigation.index(
        "if (result.wait_for(0s) == std::future_status::ready)", guarded_rejection)
    consumed_result = navigation.index("const auto terminal = result.get();", ready_branch)
    logged_terminal = navigation.index(
        "log_navigation_terminal(target, terminal.code);", consumed_result)
    ready_reject = navigation.index("return false;", logged_terminal)
    assert ready_branch < consumed_result < logged_terminal < ready_reject
    assert "cancel_navigation_goal" not in navigation[ready_branch:ready_reject]
    active_cancel = navigation.index(
        "const bool canceled = cancel_navigation_goal(", ready_reject)
    assert active_cancel > ready_reject
    assert "const bool canceled = cancel_navigation_goal(navigation_client, goal_handle, result);" in navigation

    detached_guard = navigation.index('if (require_detached && !native_attachment_state_is("detached")) {')
    detached_end = navigation.index(
        "const auto wait_status = result.wait_for(50ms);", detached_guard)
    detached_block = " ".join(navigation[detached_guard:detached_end].split())
    assert re.fullmatch(
        r'if \(require_detached && !native_attachment_state_is\("detached"\)\) '
        r'\{ RCLCPP_ERROR\([^;{}]*\); '
        r'\(void\)cancel_navigation_goal\(navigation_client, goal_handle, result\); '
        r'return false; \}', detached_block)
    simulation_limit = navigation.index("const auto simulation_limit_ns =")
    active_guard = navigation.index("while (rclcpp::ok()) {", simulation_limit)
    guard_before_wait = navigation.index(
        "if (guard && !guard()) return reject_guarded_goal();", active_guard)
    guard_after_wait = navigation.index("if (guard && !guard()) return reject_guarded_goal();", detached_end)
    ready_break = navigation.index("if (wait_status == std::future_status::ready) break;", guard_after_wait)
    assert active_guard < detached_guard < detached_end < guard_after_wait < ready_break
    assert active_guard < guard_before_wait < detached_guard
    terminal_guard = navigation.index("if (guard && !guard()) {", navigation.index("const auto wrapped = result.get();"))
    terminal_log = navigation.index("log_navigation_terminal(target, wrapped.code);", terminal_guard)
    terminal_admission_guard = navigation.index(
        "if (guard && !guard()) return false;", terminal_log)
    assert ready_break < terminal_guard < terminal_log < terminal_admission_guard
    assert "return navigate_to_with_client(" in source


def test_gate6_pickup_retreat_is_registry_bounded():
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
    assert "pregrasp.position.x = pickup_product_base[0]" in source
    assert "const double pickup_product_lateral = pickup_product_base[1]" in source
    assert "const double pickup_pedestal_lateral = pickup_pedestal_base[1]" in source
    assert '{0.85, pickup_product_lateral, 0.925}' in source
    assert '{0.90, pickup_pedestal_lateral, 0.375}' in source
    assert '{0.85, pickup_product_lateral, 0.825}' in source
    assert "wait_for_bilateral_contact(3s)" in source
    assert 'native_attachment_state_is("attached")' in source


def test_gate6_bounded_placement_translation_uses_existing_precision_route():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    alignment = _main_placement_alignment_loop(source)
    assert "node->navigate_to_aligned_precision(segment_target, 120s, true)" in alignment
    assert "node->navigate_to(segment_target, 120s)" not in alignment
    assert "achieved_segment_displacement > kMaxPlacementAlignmentSegmentDisplacement" in alignment
    assert "attachment proof failed during placement alignment" in alignment
    assert "node->navigate_to_dispatch(final_heading_target, 120s)" in source


def test_gate6_lateral_precision_translation_aligns_heading_before_travel():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    alignment = _main_placement_alignment_loop(source)
    assert "node->navigate_to_aligned_precision(segment_target, 120s, true)" in alignment
    assert "const double segment_heading" in alignment
    assert "segment_heading - current_bias_yaw" in alignment
    assert "node->navigate_to_dispatch(final_heading_target, 120s)" in source


def test_product102_precision_alignment_targets_reachable_physical_stance():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    assert "kProduct102PlacementLeadMapY" not in source
    assert "placement_alignment_target_physical = placement_alignment_physical" in source
    assert "product102_center_slot ? 0.01 : kMaxPlacementAlignmentPositionError" in source
    alignment = _main_placement_alignment_loop(source)
    assert "remaining_alignment_distance <= placement_translation_position_tolerance" in alignment
    assert "remaining_alignment_distance > placement_translation_position_tolerance" in alignment
    assert "kDesiredProduct102SlotBaseX = 0.748000000" in STANCE_HEADER.read_text()
    assert "kDesiredProduct102SlotBaseY = 0.100000000" in STANCE_HEADER.read_text()
    assert "release_radius > kMaxPlacementReleaseRadius" in source
    assert "kMaxPlacementReleaseRadius = 0.785" in source


def test_product102_precision_stance_matches_collision_verified_branch():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    # Recorded run16 geometry collided with the fixed lidar/base. The exact
    # loaded-scene replay clears this target and its 1 cm neighborhood.
    assert "kDesiredProduct102SlotBaseX = 0.748000000" in STANCE_HEADER.read_text()
    assert "kDesiredProduct102SlotBaseY = 0.100000000" in STANCE_HEADER.read_text()
    assert "kProduct102PlacementYawOffset = 1.530000000" in source
    assert "product102_center_slot ? 0.01 : kMaxPlacementAlignmentPositionError" in source
    assert "release_radius > kMaxPlacementReleaseRadius" in source
    assert "payload-aware state validity failed" in source


def test_dispatch_dock_uses_precision_arrival_before_bounded_alignment():
    source = (ROOT / "src" / "gate6_mass_stage.cpp").read_text()
    start = source.index("const std::array<double, 3> dispatch_dock_corrected_target{")
    end = source.index("const auto selected_slot", start)
    dock = source[start:end]
    assert "node->navigate_to_aligned_precision(dispatch_dock_corrected_target, 120s, true)" in dock
    assert "node->navigate_to(dispatch_dock_corrected_target, 120s)" not in dock
    assert "node->dock_pose_within_tolerance(5s)" in dock
    assert "attachment proof failed after dispatch dock" in dock
    # Exact run17 admitted dock pose still required more than 0.35 m placement
    # alignment. Closer arrival fixes the input; the bound must not be widened.
    radius = 0.785 - 0.070 - 0.005
    stance = (-4.10 + radius * 0.52 / math.hypot(0.52, 0.64),
              0.50 - radius * 0.64 / math.hypot(0.52, 0.64))
    run17_pose = (-3.3059532053619556, 0.0015578321597511837)
    assert math.hypot(run17_pose[0] + 3.4, run17_pose[1]) < 0.155
    assert math.hypot(run17_pose[0] - stance[0], run17_pose[1] - stance[1]) > 0.35
    assert math.hypot(-3.4 - stance[0], -stance[1]) + 0.01 < 0.35
    assert "kMaxPlacementAlignmentTotalDisplacement = 0.35" in source


def test_moveit_launch_retains_controller_plugin_only_for_its_process():
    source = (ROOT / "launch" / "move_group.launch.py").read_text()
    assert "additional_env={" in source
    assert 'os.environ.get("LD_PRELOAD", "")' in source
    assert '"libmoveit_simple_controller_manager.so"' in source
    assert 'os.environ["LD_PRELOAD"] =' not in source
    assert '{"use_sim_time": True}' in source
    assert '("joint_states", "/amr/base/joint_states")' in source
