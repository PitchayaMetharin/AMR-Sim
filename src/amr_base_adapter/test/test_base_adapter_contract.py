from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_adapter_forwards_only_fresh_valid_control_commands():
    source = (ROOT / "src" / "base_adapter_node.cpp").read_text()
    assert '"/amr/control/cmd_vel"' in source
    assert '"/amr/simulation/base/cmd_vel"' in source
    assert "gated_command_timeout_ms" in source
    assert "steady_clock" in source
    assert 'frame_id == "base_footprint"' in source
    assert "publish_command" in source


def test_adapter_has_no_arbitration_authority():
    source = (ROOT / "src" / "base_adapter_node.cpp").read_text()
    assert "/amr/control/cmd_vel" in source
    assert "/amr/mpc/cmd_vel" not in source
