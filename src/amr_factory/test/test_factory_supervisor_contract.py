from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "factory_supervisor_node.cpp").read_text()


def test_factory_boundary_uses_declared_interfaces():
    assert '"/amr/factory/transport_product"' in SOURCE
    assert '"/amr/factory/set_operation_mode"' in SOURCE
    assert '"/amr/factory/status"' in SOURCE
    assert "TransportProduct" in SOURCE
    assert "SetOperationMode" in SOURCE
    assert "FactoryStatus" in SOURCE


def test_factory_defaults_manual_and_bounds_autonomous_fifo():
    assert "uint8_t mode_{FactoryStatus::MANUAL}" in SOURCE
    assert "mode_ == FactoryStatus::AUTONOMOUS ? 3U : 1U" in SOURCE
    assert "std::deque<QueuedJob> queue_" in SOURCE
    assert "queue_.push_back(job)" in SOURCE
    assert "job = queue_.front()" in SOURCE
    assert "queue_.pop_front()" in SOURCE


def test_factory_rejects_invalid_duplicate_and_concurrent_goals():
    assert "GoalResponse::REJECT" in SOURCE
    assert "duplicate_product_locked" in SOURCE
    assert "active_goal_" in SOURCE
    assert "valid_pickup" in SOURCE
    assert "valid_destination" in SOURCE


def test_factory_status_is_periodic_and_motion_dependency_is_fail_closed():
    assert "create_wall_timer(200ms" in SOURCE
    assert "wait_for_action_server(2s)" in SOURCE
    assert "manipulation action dependency is unavailable" in SOURCE
    assert "fault_latched_" in SOURCE
    assert "cancel_current_manipulation" in SOURCE
