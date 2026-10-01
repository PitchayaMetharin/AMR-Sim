// Regression geometry from factory_docking_20260929_04, plan index 7.
// Published occupancy values preserve lethal/unknown classes exactly; ordinary
// inflation costs are not inferred by this minimal lethal-mask fixture.
#define main replay_backend_main
#include "planner_replay_backend.cpp"
#undef main

#include "amr_navigation/precision_navfn_planner.hpp"
#include "pluginlib/class_loader.hpp"

geometry_msgs::msg::PoseStamped stamped(const PoseData & pose)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = pose.x;
  result.pose.position.y = pose.y;
  result.pose.orientation.z = std::sin(pose.yaw * 0.5);
  result.pose.orientation.w = std::cos(pose.yaw * 0.5);
  return result;
}

std::vector<PoseData> poses(const nav_msgs::msg::Path & path)
{
  std::vector<PoseData> result;
  for (const auto & pose : path.poses) {
    const auto & q = pose.pose.orientation;
    result.push_back({pose.pose.position.x, pose.pose.position.y,
      std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)});
  }
  return result;
}

void require(bool condition, const std::string & reason)
{
  if (!condition) throw std::runtime_error(reason);
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  try {
    auto node = makeNode();
    ReplayData data{};
    data.width = 240;
    data.height = 200;
    data.resolution = 0.05;
    data.origin_x = -6.0;
    data.origin_y = -5.0;
    data.frame_id = "map";
    data.start = {1.8, 2.95, 0.0};
    data.goal = {2.4, 3.0, 0.04876021270028808};
    data.costs.assign(data.width * data.height, 0);
    for (const auto & xy : std::vector<std::pair<double, double>>{
        {0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      data.footprint.push_back(p);
    }
    // Captured pedestal mask: x=3.075..3.475, y=2.775..3.225, plus
    // the observed edge cell (3.075,2.725).
    data.costs[181 + 154 * data.width] = 254;
    for (unsigned int y = 155; y <= 164; ++y) {
      for (unsigned int x = 181; x <= 189; ++x) data.costs[x + y * data.width] = 254;
    }
    data.recorded_plan = {{"frame_id", "map"}, {"poses", json::array()}};
    data.recorded_plan["poses"].push_back(poseJson(1.8, 2.95, 0));
    for (int i = 0; i <= 20; ++i) {
      data.recorded_plan["poses"].push_back(poseJson(1.85 + i * 0.025, 2.95, 0));
    }
    data.recorded_plan["poses"].push_back(poseJson(data.goal.x, data.goal.y, data.goal.yaw));
    RunOptions options;
    options.allow_unknown = false;
    auto costmap = makeCostmap(data);
    auto baseline = recordedPath(data.recorded_plan);
    require(checkWorldPath(data, costmap.get(), baseline, false).collision_free,
      "recorded input must be footprint-clear before orientation recomputation");
    const auto broken = runConfiguredSmoother(data, options, node, baseline, "recorded_plan");
    require(broken.at("smoother_completed").get<bool>() &&
      !broken.at("collision_free").get<bool>(), "known-broken baseline must collide");

    // Optional exact captured geometry replay uses the same production seam.
    if (argc > 1) {
      data = loadSnapshot(argv[1]);
      costmap = makeCostmap(data);
    }
    auto segment = amr_navigation::make_precision_segment(
      stamped(data.start), stamped(data.goal), *costmap, data.footprint);
    require(!segment.poses.empty(), "captured direct precision segment was rejected");
    const auto candidate = runConfiguredSmoother(data, options, node, poses(segment), "precision_segment");
    require(candidate.at("success").get<bool>(), "precision segment must pass unchanged smoother and footprint sweep");
    require(candidate.at("goal_error_m").get<double>() < 1e-12 &&
      candidate.at("goal_yaw_error_rad").get<double>() < 1e-12, "exact terminal pose was changed");

    const auto reverse_start = stamped({2.4, 3.0, 0.0});
    const auto reverse_goal = stamped({1.5, 3.0, 0.0});
    auto reverse = amr_navigation::make_precision_segment(
      reverse_start, reverse_goal, *costmap, data.footprint);
    require(!reverse.poses.empty(), "registered reverse egress segment was rejected");
    for (const auto & pose : poses(reverse)) {
      require(std::abs(pose.yaw) < 1e-9, "reverse egress must retain body heading");
    }
    ReplayData reverse_data = data;
    reverse_data.start = {2.4, 3.0, 0.0};
    reverse_data.goal = {1.5, 3.0, 0.0};
    require(runConfiguredSmoother(reverse_data, options, node, poses(reverse), "reverse").at("success").get<bool>(),
      "reverse precision segment must pass unchanged smoother");

    unsigned int x, y;
    require(costmap->worldToMap(2.1, 2.975, x, y), "blocked fixture outside map");
    for (unsigned char cost : {253, 254, 255}) {
      costmap->setCost(x, y, cost);
      require(amr_navigation::make_precision_segment(
          stamped(data.start), stamped(data.goal), *costmap, data.footprint).poses.empty(),
        "inflated/blocked/unknown center must reject the direct segment");
    }
    costmap->setCost(x, y, 0);
    require(costmap->worldToMap(2.1, 3.375, x, y), "footprint fixture outside map");
    costmap->setCost(x, y, 253);
    require(!amr_navigation::make_precision_segment(
        stamped(data.start), stamped(data.goal), *costmap, data.footprint).poses.empty(),
      "footprint-only inscribed cost 253 must remain permitted");
    for (unsigned char cost : {254, 255}) {
      costmap->setCost(x, y, cost);
      require(amr_navigation::make_precision_segment(
          stamped(data.start), stamped(data.goal), *costmap, data.footprint).poses.empty(),
        "lethal/unknown footprint must reject a center-clear segment");
    }

    nav2_costmap_2d::Costmap2D turn_map(100, 100, 0.05, -2.0, -2.0, 0);
    require(turn_map.worldToMap(0.675, 0.175, x, y), "turn fixture outside map");
    turn_map.setCost(x, y, 254);
    require(amr_navigation::make_precision_segment(stamped({0, 0, 0}),
        stamped({0, 0, M_PI_2}), turn_map, data.footprint).poses.empty(),
      "endpoint-clear rotation must reject an intermediate swept collision");
    auto invalid_frame = stamped(data.goal);
    invalid_frame.header.frame_id = "odom";
    require(amr_navigation::make_precision_segment(stamped(data.start),
        invalid_frame, *costmap, data.footprint).poses.empty(), "frame mismatch accepted");

    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
    require(static_cast<bool>(loader.createSharedInstance("amr_navigation/PrecisionNavfnPlanner")),
      "precision planner plugin cannot be loaded");
    std::cout << "baseline collision reproduced; exact precision/reverse paths clear; safety negatives and plugin load passed\n";
    if (argc > 2) {
      std::ofstream(argv[2]) << json{{"baseline", broken}, {"candidate", candidate}}.dump(2) << '\n';
    }
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "precision planner contract failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
