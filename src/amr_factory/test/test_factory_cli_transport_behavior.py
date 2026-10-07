"""Classifier checks for the real CLI transport function, without ROS runtime."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def cli(monkeypatch):
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location("transport_cli_under_test", scripts / "factory_cli.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "outcome,expected_exit,expected_stdout,expected_stderr",
    [
        ("timeout", 2, "", "transport acceptance response TIMEOUT\n"),
        ("rejected", 2, "", "transport goal REJECTED\n"),
        ("delivered", 0, "delivery result\n", ""),
        ("failed", 1, "delivery result\n", ""),
        ("result_timeout", 2, "", "transport result timed out\n"),
    ],
)
def test_transport_classifies_response_without_resend_or_deadline_change(
        cli, monkeypatch, capsys, outcome, expected_exit, expected_stdout, expected_stderr):
    events = []
    goals = []
    node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
    acceptance_future, result_future = object(), object()

    class Client:
        def __init__(self, actual_node, action_type, action_name):
            assert actual_node is node
            assert action_type is cli.TransportProduct
            assert action_name == "/amr/factory/transport_product"

        def wait_for_server(self, timeout_sec):
            events.append(("server", timeout_sec))
            return True

        def send_goal_async(self, goal):
            goals.append(goal)
            return acceptance_future

    def get_result():
        events.append("get_result")
        return result_future

    accepted = SimpleNamespace(accepted=outcome != "rejected", get_result_async=get_result)

    def spin(actual_node, future, timeout):
        assert actual_node is node
        events.append(("spin", future, timeout))
        if future is acceptance_future:
            return None if outcome == "timeout" else accepted
        assert future is result_future
        return None if outcome == "result_timeout" else SimpleNamespace(
            result=SimpleNamespace(message="delivery result", delivered=outcome == "delivered"))

    def create_node(name):
        assert name == "factory_cli_transport"
        return node

    monkeypatch.setattr(cli.rclpy, "create_node", create_node)
    monkeypatch.setattr(cli, "ActionClient", Client)
    monkeypatch.setattr(cli, "_spin_until", spin)
    assert cli._transport(SimpleNamespace(
        pickup="pickup_b", destination="dispatch", timeout=240.0)) == expected_exit
    captured = capsys.readouterr()
    assert (captured.out, captured.err) == (expected_stdout, expected_stderr)
    assert len(goals) == 1
    assert (goals[0].pickup_station_id, goals[0].destination_station_id) == ("pickup_b", "dispatch")
    expected_events = [("server", 3.0), ("spin", acceptance_future, 3.0)]
    if outcome not in ("timeout", "rejected"):
        expected_events += ["get_result", ("spin", result_future, 240.0)]
    assert events == expected_events + ["destroy"]
