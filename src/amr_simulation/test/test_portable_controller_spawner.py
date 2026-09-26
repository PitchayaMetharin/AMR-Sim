"""Contract tests for the portable controller spawner wrapper."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import stat
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "portable_controller_spawner.py"


def _load():
    spec = importlib.util.spec_from_file_location("portable_controller_spawner", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_wrapper_is_executable_and_preserves_timeout_defaults():
    module = _load()
    assert stat.S_IMODE(SCRIPT.stat().st_mode) == 0o755
    assert module._timeout_values([]) == (0.0, 10.0)
    assert module._timeout_values([
        "--controller-manager-timeout", "30",
        "--service-call-timeout", "60.0",
    ]) == (30.0, 60.0)


def test_load_controller_forwards_manager_and_service_timeouts(monkeypatch):
    module = _load()
    observed = {}

    def fake_load_controller(*args, **kwargs):
        observed["args"] = args
        observed["kwargs"] = kwargs
        return "response"

    monkeypatch.setattr(module._services, "load_controller", fake_load_controller)
    result = module._load_controller_with_timeouts(
        "node",
        "/controller_manager",
        "gripper_controller",
        controller_manager_timeout=30.0,
        service_call_timeout=60.0,
    )

    assert result == "response"
    assert observed == {
        "args": ("node", "/controller_manager", "gripper_controller"),
        "kwargs": {"service_timeout": 30.0, "call_timeout": 60.0},
    }


def test_main_temporarily_patches_upstream_load_call(monkeypatch):
    module = _load()
    original_load_controller = module._spawner.load_controller
    observed = {}

    def fake_main(args=None):
        observed["load_controller"] = module._spawner.load_controller
        return 0

    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(SCRIPT),
            "gripper_controller",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "60.0",
        ],
    )
    monkeypatch.setattr(module._spawner, "main", fake_main)

    assert module.main() == 0
    assert observed["load_controller"] is not original_load_controller
    assert module._spawner.load_controller is original_load_controller
