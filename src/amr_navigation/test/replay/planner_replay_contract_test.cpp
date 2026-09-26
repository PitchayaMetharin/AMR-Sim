// Exercise the replay's conversion and safety contract without a ROS graph.
#define main replay_backend_main
#include "planner_replay_backend.cpp"
#undef main

int main()
{
  ReplayData data{};
  data.width = 100;
  data.height = 100;
  data.resolution = 0.05;
  data.origin_x = -2.0;
  data.origin_y = -2.0;
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}})
  {
    geometry_msgs::msg::Point point;
    point.x = xy.first;
    point.y = xy.second;
    data.footprint.push_back(point);
  }
  for (const unsigned char cost : {0, 253, 254, 255}) {
    data.costs.assign(data.width * data.height, cost);
    auto costmap = makeCostmap(data);
    for (bool allow_unknown : {false, true}) {
      const auto checked = checkWorldPath(data, costmap.get(), {{0, 0, 0}}, allow_unknown);
      if (checked.collision_free != (cost < 254)) {
        std::cerr << "footprint threshold mismatch at cost " << static_cast<int>(cost) << '\n';
        return 1;
      }
    }
  }
  auto costmap = makeCostmap(data);
  nav2_smac_planner::NodeLattice::CoordinateVector raw;
  raw.emplace_back(50.25F, 45.75F, 1.2F);  // goal first in A* backtrace
  raw.emplace_back(40.5F, 40.25F, -0.7F);
  const auto converted = convertLatticePath(data, costmap.get(), raw);
  if (converted.size() != 2 ||
    std::abs(converted.front().x - 0.05) > 1e-6 ||
    std::abs(converted.front().y - 0.0375) > 1e-6 ||
    std::abs(converted.front().yaw + 0.7) > 1e-6 ||
    std::abs(converted.back().x - 0.5375) > 1e-6 ||
    std::abs(converted.back().yaw - 1.2) > 1e-6)
  {
    std::cerr << "fractional-cell/radian conversion mismatch\n";
    return 1;
  }

  data.costs.assign(data.width * data.height, 0);
  auto swept_costmap = makeCostmap(data);
  unsigned int obstacle_x = 0;
  unsigned int obstacle_y = 0;
  if (!swept_costmap->worldToMap(0.675, 0.175, obstacle_x, obstacle_y)) {
    std::cerr << "swept-rotation obstacle is outside the contract map\n";
    return 1;
  }
  swept_costmap->setCost(obstacle_x, obstacle_y, 254);
  const std::vector<PoseData> rotation_endpoints{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.5 * M_PI}};
  for (const auto & endpoint : rotation_endpoints) {
    const auto endpoint_check = checkWorldPath(
      data, swept_costmap.get(), {endpoint}, false);
    if (!endpoint_check.collision_free) {
      std::cerr << "swept-rotation endpoint unexpectedly collides\n";
      return 1;
    }
  }
  const auto swept_check = checkWorldPath(
    data, swept_costmap.get(), rotation_endpoints, false);
  if (swept_check.collision_free) {
    std::cerr << "swept-rotation collision was not detected\n";
    return 1;
  }
  return 0;
}
