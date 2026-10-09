"""Exploration-only global-costmap overlay (plan C + unknown inflation).

Root-authored before implementation; the implementer must not edit this file.

Hospital run 09 evidence: Smac lattice paths overlapped cost-255 unknown
cells near approach goals because unknown space is not inflated, so Smac's
centre-cost shortcut skipped the full footprint check; the smoother's
footprint check then rejected every path (OBSTACLE_BLOCKAGE loop).
Exploration launches inflate around unknown space (and use the wider
plan-C clearance); factory and standalone launches stay byte-identical.
"""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

ROOT = Path(__file__).resolve().parents[1]
WS_SRC = ROOT.parent
LAUNCH = ROOT / "launch" / "amr_navigation.launch.py"
OVERLAY = WS_SRC / "amr_simulation" / "config" / "exploration_navigation_overlay.yaml"
PORTABLE = WS_SRC / "amr_simulation" / "launch" / "portable_exploration.launch.py"
COSTMAP_KEY = "/amr/global_costmap/global_costmap"


def _load_launch():
    spec = importlib.util.spec_from_file_location("amr_navigation_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nodes(overlay):
    description = _load_launch().generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["params_overlay"] = overlay
    pending = list(description.entities)
    nodes = []
    while pending:
        entity = pending.pop(0)
        if isinstance(entity, Node):
            nodes.append(entity)
        elif isinstance(entity, OpaqueFunction):
            pending.extend(entity.execute(context) or [])
        elif type(entity).__name__ == "TimerAction":
            pending.extend(entity.actions)
    return nodes, context


def _parameter_files(node, context):
    files = []
    for entry in node._Node__parameters:
        if isinstance(entry, ParameterFile):
            files.append("".join(
                part.perform(context) for part in entry.param_file))
        elif isinstance(entry, (str, Path)):
            files.append(str(entry))
    return files


def _by_executable(nodes, executable):
    matches = [n for n in nodes if n._Node__node_executable == executable]
    assert len(matches) == 1, executable
    return matches[0]


def test_launch_declares_params_overlay_defaulting_to_empty():
    description = _load_launch().generate_launch_description()
    arguments = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "params_overlay"]
    assert len(arguments) == 1
    default = "".join(
        part.perform(LaunchContext()) for part in arguments[0].default_value)
    assert default == ""


def test_empty_overlay_keeps_todays_parameter_files():
    nodes, context = _nodes("")
    planner = _by_executable(nodes, "planner_server")
    smoother = _by_executable(nodes, "smoother_server")
    planner_files = _parameter_files(planner, context)
    assert len(planner_files) == 1
    assert planner_files[0].endswith("/config/planner.yaml")
    assert _parameter_files(smoother, context) == planner_files
    assert any(isinstance(entry, dict) for entry in planner._Node__parameters)


def test_overlay_is_applied_after_planner_yaml_in_planner_server_only():
    nodes, context = _nodes("/tmp/overlay_example.yaml")
    planner = _by_executable(nodes, "planner_server")
    smoother = _by_executable(nodes, "smoother_server")
    files = _parameter_files(planner, context)
    assert len(files) == 2
    assert files[0].endswith("/config/planner.yaml")
    assert files[1] == "/tmp/overlay_example.yaml"
    # The global costmap lives in planner_server; the smoother only reads its topic.
    assert len(_parameter_files(smoother, context)) == 1


def test_overlay_file_only_widens_clearance_and_inflates_around_unknown():
    overlay = yaml.safe_load(OVERLAY.read_text())
    assert set(overlay) == {COSTMAP_KEY}
    parameters = overlay[COSTMAP_KEY]["ros__parameters"]
    assert set(parameters) == {"inflation_layer"}
    inflation = parameters["inflation_layer"]
    assert inflation == {
        "inflation_radius": 1.0,
        "cost_scaling_factor": 2.0,
        "inflate_around_unknown": True,
    }


def test_overlay_targets_the_global_costmap_node_used_by_planner_yaml():
    planner = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    assert COSTMAP_KEY in planner
    costmap = planner[COSTMAP_KEY]["ros__parameters"]
    assert "inflation_layer" in costmap["plugins"]
    footprint = yaml.safe_load(costmap["footprint"])
    padding = costmap.get("footprint_padding", 0.0)
    circumscribed = max(
        math.hypot(x + math.copysign(padding, x), y + math.copysign(padding, y))
        for x, y in footprint)
    overlay = yaml.safe_load(OVERLAY.read_text())
    radius = overlay[COSTMAP_KEY]["ros__parameters"]["inflation_layer"]["inflation_radius"]
    # Smac's centre-cost shortcut is only sound if inflation covers the
    # circumscribed radius (now also around unknown space).
    assert radius >= circumscribed


def test_factory_planner_yaml_inflation_is_unchanged():
    planner = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    inflation = planner[COSTMAP_KEY]["ros__parameters"]["inflation_layer"]
    assert inflation["inflation_radius"] == 0.75
    assert inflation["cost_scaling_factor"] == 3.0
    assert "inflate_around_unknown" not in inflation


def test_portable_exploration_passes_the_installed_overlay():
    source = PORTABLE.read_text()
    start = source.index('navigation_include = _package_launch_include(')
    end = source.index('mpc_include = _package_launch_include(', start)
    call = source[start:end]
    assert '"params_overlay"' in call
    assert "exploration_navigation_overlay.yaml" in source
    assert 'get_package_share_directory("amr_simulation")' in source or \
        "get_package_share_directory('amr_simulation')" in source


def test_overlay_is_installed_with_amr_simulation():
    cmake = (WS_SRC / "amr_simulation" / "CMakeLists.txt").read_text()
    install_lines = [
        line for line in cmake.splitlines() if line.strip().startswith("install(")]
    assert any("config" in line for line in install_lines) or \
        "exploration_navigation_overlay.yaml" in cmake
