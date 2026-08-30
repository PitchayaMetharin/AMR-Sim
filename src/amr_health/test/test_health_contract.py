from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_health_supervisor_is_observational():
    source = (ROOT / "src" / "health_supervisor_node.cpp").read_text()
    assert "create_publisher<HealthStatus>" in source
    assert '"/amr/health/status"' in source
    for forbidden in (
        "cmd_vel",
        "change_state",
    ):
        assert forbidden not in source


def test_health_supervisor_checks_base_evidence():
    source = (ROOT / "src" / "health_supervisor_node.cpp").read_text()
    assert '"/amr/base/status"' in source
    assert "std::chrono::steady_clock" in source
    assert "InvalidCause::BACKWARD_TIME" in source
    assert "sequence > evidence.sequence" in source
