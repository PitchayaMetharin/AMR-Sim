from pathlib import Path
import copy
import importlib.util
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]


def _launch_module():
    spec = importlib.util.spec_from_file_location(
        "mpc_launch_contract", ROOT / "launch" / "amr_mpc_controller.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _registries(tmp_path):
    factory = ROOT.parent / "amr_factory" / "config"
    products = yaml.safe_load((factory / "products.yaml").read_text())
    stations = yaml.safe_load((factory / "stations.yaml").read_text())
    paths = tmp_path / "products.yaml", tmp_path / "stations.yaml"
    return products, stations, paths


def _actions(module, monkeypatch, enabled, paths):
    context = LaunchContext()
    context.launch_configurations.update(
        controller_frequency="20.0", enable_final_position_profiles=enabled,
        products_registry=str(paths[0]), stations_registry=str(paths[1]))
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(ROOT))
    calls = []
    original = module.Node

    def capture(**kwargs):
        calls.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, "Node", capture)
    return context, calls


def test_disabled_profile_builder_reads_no_files_and_preserves_launch(monkeypatch, tmp_path):
    module = _launch_module()
    monkeypatch.setattr(module, "_registry", lambda _: pytest.fail("Disabled profile read a registry"))
    context, calls = _actions(module, monkeypatch, "false", (tmp_path / "missing", tmp_path / "missing2"))
    actions = module._launch_controller(context)
    assert len(calls) == 2
    assert calls[0]["parameters"][-1] == {}
    assert calls[0]["remappings"] == [("cmd_vel", "/amr/mpc/cmd_vel")]
    assert calls[1]["parameters"] == [str(ROOT / "config" / "controller.yaml")]
    assert actions[0].__class__.__name__ == "Node"
    assert actions[1].__class__.__name__ == "TimerAction"
    assert actions[1].period == 1.0
    baseline = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    assert baseline["/amr/controller_server"]["ros__parameters"]["controller_plugins"] == ["FollowPath", "PlacementFollowPath"]


def test_enabled_builder_executes_with_raw_registry_and_independent_profiles(monkeypatch, tmp_path):
    module = _launch_module()
    products, stations, paths = _registries(tmp_path)
    for value, path in zip((products, stations), paths):
        path.write_text(yaml.safe_dump(value))
    context, calls = _actions(module, monkeypatch, "true", paths)
    module._launch_controller(context)
    assert len(calls) == 2
    overrides = calls[0]["parameters"][-1]
    assert overrides["controller_plugins"] == [
        "FollowPath", "PlacementFollowPath", "FinalPositionFollowPathA",
        "FinalPositionPlacementFollowPathA", "FinalPositionFollowPathB",
        "FinalPositionPlacementFollowPathB"]
    assert overrides["final_position_profiles.dispatch_dock"] == [-3.4, 0.0, 3.141592653589793]
    assert overrides["final_position_profiles.slot_a"] == [-4.1, 0.5, 0.075]
    assert overrides["final_position_profiles.slot_b"] == [-4.1, 0.0, 0.075]
    baseline = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())["/amr/controller_server"]["ros__parameters"]
    plugins = {c.attrib["name"] for c in ET.parse(ROOT / "heading_latched_rpp_plugin.xml").getroot().findall("class")}
    for suffix in ("A", "B"):
        for base, prefix, plugin in (
            ("FollowPath", "FinalPositionFollowPath", "amr_mpc_controller::FinalPositionRPP"),
            ("PlacementFollowPath", "FinalPositionPlacementFollowPath", "amr_mpc_controller::FinalPositionPlacementRPP"),
        ):
            expected = copy.deepcopy(baseline[base])
            expected.update(plugin=plugin, desired_linear_vel=0.5)
            assert expected["approach_velocity_scaling_dist"] == 0.60
            assert overrides[prefix + suffix] == expected
            assert plugin in plugins
    overrides["FinalPositionFollowPathA"]["cost_scaling_dist"] = -1
    assert overrides["FinalPositionFollowPathB"]["cost_scaling_dist"] == baseline["FollowPath"]["cost_scaling_dist"]
    assert baseline["FollowPath"]["desired_linear_vel"] == 0.5
    assert baseline["PlacementFollowPath"]["desired_linear_vel"] == 0.1


@pytest.mark.parametrize("defect", [
    "wrong_id", "bool_id", "duplicate_id", "wrong_slot", "shared_slot", "duplicate_slot",
    "inactive", "pickup_missing", "products_frame", "stations_frame", "nan_dock", "nan_slot",
    "bool_coordinate", "missing_coordinate", "missing_dispatch", "missing_file", "nonmapping", "malformed",
])
def test_invalid_profile_registry_fails_before_node_construction(monkeypatch, tmp_path, defect):
    module = _launch_module()
    products, stations, paths = _registries(tmp_path)
    a, b = products["products"]["product_a"], products["products"]["product_b"]
    if defect == "wrong_id": a["tag_id"] = 104
    elif defect == "bool_id": a["tag_id"] = True
    elif defect == "duplicate_id": b["tag_id"] = a["tag_id"]
    elif defect == "wrong_slot": a["dispatch_slot"], b["dispatch_slot"] = b["dispatch_slot"], a["dispatch_slot"]
    elif defect == "shared_slot": b["dispatch_slot"] = a["dispatch_slot"]
    elif defect == "duplicate_slot": products["dispatch_slots"].append(copy.deepcopy(products["dispatch_slots"][0]))
    elif defect == "inactive": a["autonomous_enabled"] = False
    elif defect == "pickup_missing": a["pickup_station"] = "absent"
    elif defect == "products_frame": products["frame_id"] = "odom"
    elif defect == "stations_frame": stations["frame_id"] = "odom"
    elif defect == "nan_dock": stations["stations"]["dispatch"]["dock"]["yaw"] = float("nan")
    elif defect == "nan_slot": products["dispatch_slots"][1]["z"] = float("nan")
    elif defect == "bool_coordinate": products["dispatch_slots"][0]["x"] = True
    elif defect == "missing_coordinate": del products["dispatch_slots"][0]["z"]
    elif defect == "missing_dispatch": del stations["stations"]["dispatch"]
    for value, path in zip((products, stations), paths):
        path.write_text(yaml.safe_dump(value))
    if defect == "missing_file": paths[0].unlink()
    elif defect == "nonmapping": paths[0].write_text("[]")
    elif defect == "malformed": paths[0].write_text("products: [")
    context, calls = _actions(module, monkeypatch, "true", paths)
    with pytest.raises((ValueError, OSError, yaml.YAMLError)):
        module._launch_controller(context)
    assert calls == []


def test_registry_dependency_is_yaml_without_factory_cycle():
    package = ET.parse(ROOT / "package.xml").getroot()
    assert "python3-yaml" in [e.text for e in package.findall("exec_depend")]
    assert not any(e.text == "amr_factory" for e in package)


def test_rpp_is_direct_and_within_simulation_limits():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/amr/localization/wheel_odometry"
    plugin = controller["FollowPath"]
    assert plugin["plugin"] == "amr_mpc_controller::HeadingLatchedRPP"
    assert "primary_controller" not in plugin
    assert plugin["desired_linear_vel"] == 0.50
    assert plugin["lookahead_dist"] == 0.60
    assert plugin["min_lookahead_dist"] == 0.30
    assert plugin["max_lookahead_dist"] == 0.90
    assert plugin["lookahead_time"] == 1.50
    assert plugin["use_velocity_scaled_lookahead_dist"] is True
    assert plugin["transform_tolerance"] == 0.30
    assert plugin["use_rotate_to_heading"] is True
    assert plugin["rotate_to_heading_angular_vel"] == 0.64
    assert plugin["max_angular_accel"] == 0.40
    assert plugin["rotate_to_heading_min_angle"] == 0.785
    assert plugin["allow_reversing"] is False
    assert plugin["use_interpolation"] is True


def test_rpp_regulation_thresholds_are_explicit_and_fail_closed():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    plugin = config["/amr/controller_server"]["ros__parameters"]["FollowPath"]
    assert plugin["min_approach_linear_velocity"] == 0.05
    assert plugin["approach_velocity_scaling_dist"] == 0.60
    assert plugin["use_regulated_linear_velocity_scaling"] is True
    assert plugin["regulated_linear_scaling_min_radius"] == 0.90
    assert plugin["regulated_linear_scaling_min_speed"] == 0.15
    assert plugin["use_cost_regulated_linear_velocity_scaling"] is True
    assert plugin["cost_scaling_dist"] > 0.0
    assert plugin["cost_scaling_gain"] > 0.0
    assert plugin["inflation_cost_scaling_factor"] > 0.0
    assert plugin["use_collision_detection"] is True
    assert plugin["max_allowed_time_to_collision_up_to_carrot"] > 0.0


def test_placement_controller_is_collision_checked_and_can_back_away():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["controller_plugins"] == ["FollowPath", "PlacementFollowPath"]
    normal = controller["FollowPath"]
    placement = controller["PlacementFollowPath"]
    assert placement["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
    assert placement["desired_linear_vel"] == 0.10
    assert placement["use_collision_detection"] is True
    assert placement["min_approach_linear_velocity"] == 0.02
    assert placement["allow_reversing"] is True
    assert placement["use_rotate_to_heading"] is False
    assert placement["rotate_to_heading_angular_vel"] == 0.64
    assert placement["max_angular_accel"] == 0.40
    assert normal["allow_reversing"] is False
    assert normal["use_rotate_to_heading"] is True


def test_retreat_goal_checker_is_xy_only_and_tight():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    assert controller["goal_checker_plugins"] == [
        "goal_checker", "placement_goal_checker", "retreat_goal_checker"]
    checker = controller["retreat_goal_checker"]
    assert checker["plugin"] == "nav2_controller::PositionGoalChecker"
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == 0.01
    assert "yaw_goal_tolerance" not in checker


def test_progress_checker_counts_deliberate_diff_drive_rotation():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["progress_checker"]
    assert checker["plugin"] == "nav2_controller::PoseProgressChecker"
    assert checker["required_movement_radius"] == 0.20
    assert checker["required_movement_angle"] == 0.20
    assert checker["movement_time_allowance"] == 10.0


def test_localized_goal_window_leaves_margin_for_independent_dock_acceptance():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["goal_checker"]
    assert checker["xy_goal_tolerance"] == 0.07
    assert checker["yaw_goal_tolerance"] == 0.15
    assert controller["goal_checker_plugins"] == [
        "goal_checker", "placement_goal_checker", "retreat_goal_checker"]


def test_precise_goal_checker_is_private_and_tightens_xy_only():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    controller = config["/amr/controller_server"]["ros__parameters"]
    checker = controller["placement_goal_checker"]
    assert checker["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert checker["stateful"] is False
    assert checker["xy_goal_tolerance"] == 0.01
    assert checker["yaw_goal_tolerance"] == 0.15


def test_regulated_pure_pursuit_dependency_is_declared():
    package = (ROOT / "package.xml").read_text()
    assert "<depend>nav2_regulated_pure_pursuit_controller</depend>" in package
    assert "nav2_mppi_controller" not in package
    assert "nav2_rotation_shim_controller" not in package


def test_local_costmap_uses_local_state_and_both_perception_clouds():
    config = yaml.safe_load((ROOT / "config" / "controller.yaml").read_text())
    costmap = config["/amr/local_costmap/local_costmap"]["ros__parameters"]
    assert costmap["global_frame"] == "odom"
    assert costmap["robot_base_frame"] == "base_footprint"
    assert costmap["inflation_layer"]["inflation_radius"] >= 0.41
    obstacle = costmap["obstacle_layer"]
    assert obstacle["front_points"]["topic"] == "/amr/perception/front_lidar/points"
    assert obstacle["rear_points"]["topic"] == "/amr/perception/rear_lidar/points"


def test_controller_output_is_internal_to_arbitration():
    launch = (ROOT / "launch" / "amr_mpc_controller.launch.py").read_text()
    assert '("cmd_vel", "/amr/mpc/cmd_vel")' in launch
    assert 'DeclareLaunchArgument("controller_frequency", default_value="20.0")' in launch
    assert '"controller_frequency": ParameterValue(' in launch
    assert 'LaunchConfiguration("controller_frequency"), value_type=float' in launch


def test_lifecycle_manager_starts_after_controller_construction_barrier():
    launch = (ROOT / "launch" / "amr_mpc_controller.launch.py").read_text()
    assert "controller_server = Node(" in launch
    assert "lifecycle_manager = Node(" in launch
    assert "TimerAction(period=1.0, actions=[lifecycle_manager])" in launch
