from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_manipulation_action_boundary_is_owned_and_fail_closed():
    source = (ROOT / "src" / "manipulation_supervisor_node.cpp").read_text()
    assert '"/amr/manipulation/manipulate_product"' in source
    assert "Action::Goal::PICK" in source
    assert "Action::Goal::PLACE" in source
    assert 'goal.station_id == "dispatch"' in source
    assert 'product_id == "101"' in source
    assert "active_goal_" in source
    assert "GoalResponse::REJECT" in source
    assert "cancel_requested_.store(true)" in source
    assert "Gate 6 executor hook is unavailable; no motion dispatched" in source


def test_manipulation_status_blocks_base_during_active_or_failed_work():
    source = (ROOT / "src" / "manipulation_supervisor_node.cpp").read_text()
    assert '"/amr/manipulation/status"' in source
    assert "create_wall_timer(50ms" in source
    assert "state_ = Status::MOVING" in source
    assert "base_motion_allowed_ = false" in source
    assert "state_ = Status::FAULT" in source
    assert "canceled with product retained; base motion blocked" in source
    assert "goal_handle->canceled" in source
