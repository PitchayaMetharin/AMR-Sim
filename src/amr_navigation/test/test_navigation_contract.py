import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _max_footprint_radius(footprint, padding=0.0):
    points = yaml.safe_load(footprint) if isinstance(footprint, str) else footprint
    return max(
        math.hypot(
            point[0] + math.copysign(padding, point[0]),
            point[1] + math.copysign(padding, point[1]),
        )
        for point in points
    )


def test_planner_consumes_map_and_independent_perception_clouds():
    config = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    planner = config["/amr/planner_server"]["ros__parameters"]
    costmap = config["/amr/global_costmap/global_costmap"]["ros__parameters"]
    assert planner["planner_plugins"] == ["GridBased", "PrecisionGridBased"]
    grid_based = planner["GridBased"]
    assert grid_based["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert grid_based["tolerance"] == 0.05
    assert grid_based["allow_unknown"] is False
    assert grid_based["analytic_expansion_max_length"] == 0.0
    assert grid_based["smooth_path"] is False
    assert grid_based["allow_reverse_expansion"] is False
    assert grid_based["lookup_table_size"] == 20.0
    assert grid_based["max_iterations"] == -1
    assert grid_based["max_on_approach_iterations"] == 1000
    assert grid_based["max_planning_time"] == 2.0
    assert grid_based["cost_penalty"] == 2.0
    assert grid_based["analytic_expansion_ratio"] == 3.5
    assert grid_based["change_penalty"] == 0.05
    assert grid_based["non_straight_penalty"] == 1.05
    assert grid_based["reverse_penalty"] == 2.0
    assert grid_based["retrospective_penalty"] == 0.015
    assert grid_based["rotation_penalty"] == 5.0
    assert grid_based["cache_obstacle_heuristic"] is False
    assert set(grid_based) == {
        "plugin", "tolerance", "allow_unknown", "max_iterations",
        "max_on_approach_iterations", "max_planning_time",
        "analytic_expansion_max_length", "analytic_expansion_ratio",
        "smooth_path", "lookup_table_size", "cost_penalty", "change_penalty",
        "non_straight_penalty", "reverse_penalty", "retrospective_penalty",
        "rotation_penalty", "allow_reverse_expansion", "cache_obstacle_heuristic",
    }
    precision_grid_based = planner["PrecisionGridBased"]
    assert precision_grid_based["plugin"] == "amr_navigation/PrecisionNavfnPlanner"
    assert precision_grid_based["use_astar"] is True
    assert precision_grid_based["allow_unknown"] is False
    assert precision_grid_based["tolerance"] == 0.01
    assert costmap["global_frame"] == "map"
    assert costmap["robot_base_frame"] == "base_footprint"
    assert costmap["static_layer"]["map_topic"] == "/map"
    assert costmap["footprint_padding"] == 0.01
    assert costmap["inflation_layer"]["inflation_radius"] >= _max_footprint_radius(
        costmap["footprint"], costmap["footprint_padding"])
    obstacle = costmap["obstacle_layer"]
    assert obstacle["front_points"]["topic"] == "/amr/perception/front_lidar/points"
    assert obstacle["rear_points"]["topic"] == "/amr/perception/rear_lidar/points"


def test_launch_has_planner_but_no_motion_runtime():
    launch = (ROOT / "launch" / "amr_navigation.launch.py").read_text()
    assert 'executable="planner_server"' in launch
    assert 'executable="smoother_server"' in launch
    assert 'get_package_share_directory("nav2_smac_planner")' in launch
    assert '"5cm_resolution", "0.5m_turning_radius", "diff", "output.json"' in launch
    assert '"GridBased.lattice_filepath": lattice_file' in launch
    assert '<exec_depend>nav2_smac_planner</exec_depend>' in (ROOT / "package.xml").read_text()
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
    assert plugin["w_smooth"] == 0.0
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


def test_local_inflation_covers_the_shared_padded_footprint():
    planner = yaml.safe_load((ROOT / "config" / "planner.yaml").read_text())
    mpc = yaml.safe_load(
        (ROOT.parent / "amr_mpc_controller" / "config" / "controller.yaml").read_text())
    planner_costmap = planner["/amr/global_costmap/global_costmap"]["ros__parameters"]
    local_costmap = mpc["/amr/local_costmap/local_costmap"]["ros__parameters"]
    assert local_costmap["footprint_padding"] == planner_costmap["footprint_padding"]
    required_radius = _max_footprint_radius(
        planner_costmap["footprint"], planner_costmap["footprint_padding"])
    assert local_costmap["inflation_layer"]["inflation_radius"] >= required_radius
