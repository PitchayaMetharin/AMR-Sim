from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_mission_boundary_is_managed_and_validated():
    source = (ROOT / "src" / "mission_supervisor_node.cpp").read_text()
    assert '"/amr/mission/navigate_to_pose"' in source
    assert '"/amr/mission/navigate_to_pose_precise"' in source
    assert '"/amr/mission/navigate_to_pose_retreat"' in source
    assert '"retreat_goal_checker"' in source
    assert '"placement_goal_checker"' in source
    assert 'frame_id != "map"' in source
    assert "PRIMARY_STATE_ACTIVE" in source
    assert "GoalResponse::REJECT" in source
    assert "mission->is_active()" in source
    assert "behavior_tree.empty()" in source


def test_mission_sequences_planning_then_path_following():
    source = (ROOT / "src" / "mission_supervisor_node.cpp").read_text()
    assert '"/amr/compute_path_to_pose"' in source
    assert '"/amr/smooth_path"' in source
    assert '"/amr/follow_path"' in source
    assert "start_planning" in source
    assert "start_smoothing" in source
    assert "start_following" in source
    assert "path.poses.empty()" in source
    assert 'goal.goal_checker_id = goal_checker_id' in source
    assert 'goal.controller_id = controller_id' in source
    assert '"PlacementFollowPath"' in source


def test_mission_endpoints_share_one_reserved_goal_lifecycle():
    source = (ROOT / "src" / "mission_supervisor_node.cpp").read_text()
    assert "create_mission_server" in source
    assert "precise_server_" in source
    assert "retreat_server_" in source
    assert "goal_reserved_" in source
    assert "reserved_goal_checker_id_" in source
    assert "reserved_controller_id_" in source
    assert "mission_goal_checker_id_" in source
    assert "mission_controller_id_" in source
    assert "goal_reserved_ || state_ != MissionState::IDLE || mission_goal_" in source
    assert "mission_goal_checker_id_.clear();" in source
    assert "reserved_goal_checker_id_.clear();" in source
    assert "reserved_controller_id_.clear();" in source
    assert "mission_controller_id_.clear();" in source


def test_mission_uses_explicit_race_safe_lifecycle():
    source = (ROOT / "src" / "mission_supervisor_node.cpp").read_text()
    for state in (
            "IDLE", "PLANNER_PENDING", "PLANNER_ACTIVE", "SMOOTHER_PENDING",
            "SMOOTHER_ACTIVE", "CONTROLLER_PENDING", "CONTROLLER_ACTIVE",
            "CANCELING"):
        assert state in source
    assert "cancel_requested_" in source
    assert "terminal_reported_" in source
    assert "UnknownGoalHandleError" in source
    assert "planner_goal_.reset();" in source
    assert "smoother_goal_.reset();" in source
    assert "controller_goal_.reset();" in source
    assert "mission_feedback->current_pose" in source
    assert "mission_feedback->navigation_time = ros_now - mission_start_" in source
    assert "last_ros_time_" in source
    assert "tf_buffer_.lookupTransform(\n        \"map\", \"base_footprint\"" in source


def test_mission_has_no_direct_motion_or_permission_authority():
    source = (ROOT / "src" / "mission_supervisor_node.cpp").read_text()
    assert "cmd_vel" not in source
