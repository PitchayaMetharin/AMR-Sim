from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_planner_consumes_map_and_independent_perception_clouds():
    config = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    planner = config["/amr/planner_server"]["ros__parameters"]
    costmap = config["/amr/global_costmap/global_costmap"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased"]
    assert planner["GridBased"]["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert planner["GridBased"]["tolerance"] == 0.05
    assert costmap["global_frame"] == "map"
    assert costmap["robot_base_frame"] == "base_footprint"
    assert costmap["static_layer"]["map_topic"] == "/map"
    assert costmap["inflation_layer"]["inflation_radius"] >= 0.41
    obstacle = costmap["obstacle_layer"]
    assert obstacle["front_points"]["topic"] == "/amr/perception/front_lidar/points"
    assert obstacle["rear_points"]["topic"] == "/amr/perception/rear_lidar/points"


def test_launch_has_planner_but_no_motion_runtime():
    launch = (ROOT / "launch" / "amr_navigation.launch.py").read_text()
    assert 'executable="planner_server"' in launch
    assert 'executable="smoother_server"' in launch
    for forbidden in ("controller_server", "bt_navigator", "behavior_server",
                      "velocity_smoother", "cmd_vel"):
        assert forbidden not in launch


def test_lifecycle_manager_starts_after_planning_construction_barrier():
    launch = (ROOT / "launch" / "amr_navigation.launch.py").read_text()
    barrier = "TimerAction(period=1.0, actions=[lifecycle_manager])"
    assert "lifecycle_manager = Node(" in launch
    assert barrier in launch
    barrier_index = launch.index(barrier)
    assert launch.index('executable="planner_server"') < barrier_index
    assert launch.index('executable="smoother_server"') < barrier_index


def test_smoother_is_collision_checked_and_lifecycle_managed():
    config = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    smoother = config["/amr/smoother_server"]["ros__parameters"]
    assert smoother["smoother_plugins"] == ["simple_smoother"]
    plugin = smoother["simple_smoother"]
    assert plugin["plugin"] == "nav2_smoother::SimpleSmoother"
    assert plugin["w_data"] == 0.2
    assert plugin["w_smooth"] == 0.3
    assert plugin["do_refinement"] is True
    lifecycle = config["/amr/lifecycle_manager_planning"]["ros__parameters"]
    assert lifecycle["node_names"] == ["planner_server", "smoother_server"]
    assert smoother["costmap_topic"] == "global_costmap/costmap_raw"
    assert smoother["footprint_topic"] == "global_costmap/published_footprint"


def test_egress_footprint_matches_both_nav2_footprints():
    control = yaml.safe_load(
        (ROOT.parent / "amr_control" / "config" / "control.yaml").read_text())
    planner = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    mpc = yaml.safe_load(
        (ROOT.parent / "amr_mpc_controller" / "config" / "controller.yaml").read_text())
    egress = control["/amr/command_arbitration_node"]["ros__parameters"][
        "egress_footprint"]
    assert egress == planner["/amr/global_costmap/global_costmap"]["ros__parameters"][
        "footprint"]
    assert egress == mpc["/amr/local_costmap/local_costmap"]["ros__parameters"][
        "footprint"]
