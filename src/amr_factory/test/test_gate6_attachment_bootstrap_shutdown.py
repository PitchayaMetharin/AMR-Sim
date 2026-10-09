import importlib.util
import sys
from pathlib import Path

import pytest
import rclpy

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "gate6_attachment_bootstrap.py"


class _FakeBootstrap:
    def run(self):
        pass

    def fail_closed(self, detail):
        raise AssertionError(f"unexpected fail_closed: {detail}")

    def destroy_node(self):
        pass


@pytest.fixture
def module(monkeypatch):
    spec = importlib.util.spec_from_file_location("gate6_attachment_bootstrap_under_test", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, mod)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, "AttachmentBootstrap", _FakeBootstrap)
    yield mod
    rclpy.try_shutdown()


def test_spin_error_after_context_shutdown_is_orderly(module, monkeypatch):
    def spin(_node):
        rclpy.shutdown()
        raise RuntimeError("Unable to convert call argument to Python object")

    monkeypatch.setattr(module.rclpy, "spin", spin)
    assert module.main() == 0


def test_spin_error_with_live_context_fails_closed(module, monkeypatch, capsys):
    def spin(_node):
        raise RuntimeError("Unable to convert call argument to Python object")

    monkeypatch.setattr(module.rclpy, "spin", spin)
    assert module.main() == 1
    assert "GATE6 ATTACHMENT BOOTSTRAP FAULT: Unable to convert" in capsys.readouterr().out
