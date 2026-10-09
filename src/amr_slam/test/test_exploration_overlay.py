"""Exploration scan-density overlay contracts, authored before implementation.

Build launch descriptions with real ROS launch objects, without executing any
processes. Shared mapper and standalone SLAM defaults remain unchanged.
"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
import yaml


ROOT = Path(__file__).resolve().parents[1]
SIMULATION = ROOT.parent / "amr_simulation"
SLAM_LAUNCH = ROOT / "launch" / "amr_slam.launch.py"
PORTABLE_LAUNCH = SIMULATION / "launch" / "portable_exploration.launch.py"
OVERLAY = SIMULATION / "config" / "exploration_slam_overlay.yaml"


def _load_launch(path):
    spec = importlib.util.spec_from_file_location("scan_density_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _slam_nodes(overlay=None):
    description = _load_launch(SLAM_LAUNCH).generate_launch_description()
    context = LaunchContext()
    if overlay is not None:
        context.launch_configurations["params_overlay"] = str(overlay)
    nodes = []
    pending = list(description.entities)
    while pending:
        entity = pending.pop(0)
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            pending.extend(entity.execute(context) or [])
        elif isinstance(entity, Node):
            nodes.append(entity)
        else:
            raise AssertionError("unexpected standalone SLAM action: %r" % entity)
    return nodes, context


def _parameter_files(node, context):
    files = []
    for entry in node._Node__parameters:
        if isinstance(entry, ParameterFile):
            files.append("".join(part.perform(context) for part in entry.param_file))
        elif isinstance(entry, (str, Path)):
            files.append(str(entry))
        else:
            raise AssertionError("unexpected SLAM parameter entry: %r" % entry)
    return files


def _assert_single_async_slam(nodes):
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == "slam_toolbox"
    assert node.node_executable == "async_slam_toolbox_node"
    assert node._Node__node_name == "slam_toolbox"
    assert node._Node__node_namespace == "/amr"
    return node


def test_params_overlay_is_optional_and_defaults_to_empty():
    description = _load_launch(SLAM_LAUNCH).generate_launch_description()
    arguments = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == "params_overlay"]
    assert len(arguments) == 1
    assert "".join(
        part.perform(LaunchContext()) for part in arguments[0].default_value) == ""


def test_default_and_explicit_empty_keep_only_installed_mapper_parameters():
    mapper = str(Path(get_package_share_directory("amr_slam")) / "config" / "mapper.yaml")
    for overlay in (None, ""):
        nodes, context = _slam_nodes(overlay)
        node = _assert_single_async_slam(nodes)
        assert _parameter_files(node, context) == [mapper]


def test_supplied_overlay_is_appended_after_installed_mapper(tmp_path):
    supplied = tmp_path / "supplied_overlay.yaml"
    supplied.write_text("/amr/slam_toolbox:\n  ros__parameters: {}\n")
    nodes, context = _slam_nodes(supplied)
    node = _assert_single_async_slam(nodes)
    mapper = str(Path(get_package_share_directory("amr_slam")) / "config" / "mapper.yaml")
    assert _parameter_files(node, context) == [mapper, str(supplied)]


def test_exploration_overlay_changes_only_the_three_sampling_parameters():
    assert yaml.safe_load(OVERLAY.read_text()) == {
        "/amr/slam_toolbox": {
            "ros__parameters": {
                "minimum_time_interval": 0.5,
                "minimum_travel_distance": 0.3,
                "minimum_travel_heading": 0.3,
            },
        },
    }


def test_portable_runtime_passes_installed_overlay_to_actual_slam_include(monkeypatch):
    portable = _load_launch(PORTABLE_LAUNCH)
    monkeypatch.setattr(
        portable.xacro, "process_file",
        lambda *_args, **_kwargs: SimpleNamespace(
            toxml=lambda: "<robot name='amr'><link name='base_link'/></robot>"))
    monkeypatch.setattr(portable, "_gazebo_runtime_actions", lambda *_args, **_kwargs: [])
    calls = []
    original_include = portable._package_launch_include

    def capture_include(package, *, arguments=None):
        action = original_include(package, arguments=arguments)
        calls.append((package, arguments, action))
        return action

    monkeypatch.setattr(portable, "_package_launch_include", capture_include)
    context = LaunchContext()
    context.launch_configurations.update({
        "localization_mode": "slam",
        "headless": "true",
        "software_rendering": "false",
    })
    world = portable.ValidatedWorld(SIMULATION / "worlds" / "amr_world.sdf", "amr_world")
    # Construct the actual graph; never execute Node or process actions.
    portable._runtime_actions(context, world, (0.0, 0.0, 0.12, 0.0))
    slam_calls = [call for call in calls if call[0] == "amr_slam"]
    assert len(slam_calls) == 1
    expected = str(
        Path(get_package_share_directory("amr_simulation"))
        / "config" / "exploration_slam_overlay.yaml")
    assert slam_calls[0][1] == {"params_overlay": expected}
    # Also inspect the real IncludeLaunchDescription created by the wrapper.
    assert dict(slam_calls[0][2].launch_arguments) == {"params_overlay": expected}
