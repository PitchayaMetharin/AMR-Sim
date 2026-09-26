// Offline replay of the installed Nav2 Smac planner cores and the configured
// SimpleSmoother against a finalized runtime snapshot.
//
// This is intentionally a standalone test harness.  It does not change the
// navigation package dependencies or planner configuration until the replay
// has demonstrated the required behavior.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/constants.hpp"
#ifndef AMR_REPLAY_LATTICE_ONLY
#include "nav2_smac_planner/node_2d.hpp"
#endif
#ifndef AMR_REPLAY_2D_ONLY
#include "nav2_smac_planner/node_lattice.hpp"
#endif
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smoother/simple_smoother.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{

using json = nlohmann::json;
using nav2_smac_planner::GridCollisionChecker;
using nav2_smac_planner::MotionModel;
using nav2_smac_planner::SearchInfo;

constexpr double kGoalPositionTolerance = 0.07;
constexpr double kGoalYawTolerance = 0.15;
constexpr double kPathPlanningTime = 2.0;
constexpr int kMaxOnApproachIterations = 1000;
constexpr float kLookupTableSize = 20.0F;
constexpr int kEndpointBlockRadiusCells = 2;
constexpr unsigned int kLatticeCollisionBins = 72;
constexpr double kLatticeLookupTableMeters = 20.0;
// Candidate policy: disable analytic shortcuts. Humble checks fractional
// shortcut poses at truncated cells, which admitted lethal overlaps in Run07.
constexpr double kLatticeAnalyticExpansionMaxLengthMeters = 0.0;
constexpr double kFootprintSampleSpacingFraction = 0.5;

struct PoseData
{
  double x;
  double y;
  double yaw;
};

struct ReplayData
{
  unsigned int width;
  unsigned int height;
  double resolution;
  double origin_x;
  double origin_y;
  std::string frame_id;
  std::vector<unsigned char> costs;
  nav2_costmap_2d::Footprint footprint;
  PoseData start;
  PoseData goal;
  json recorded_plan;
};

struct RunOptions
{
  std::string snapshot;
  std::string lattice_file;
  std::string planner = "all";
  std::string block_endpoint = "none";
  std::string unknown_endpoint = "none";
  bool allow_unknown = true;
  bool run_smoother = false;
  std::string output;
};

double wrapAngle(double value)
{
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

PoseData readPose(const json & value, const std::string & label)
{
  if (!value.is_object() || !value.contains("x") || !value.contains("y") ||
    !value.contains("yaw"))
  {
    throw std::runtime_error("snapshot pose is incomplete: " + label);
  }
  return PoseData{
    value.at("x").get<double>(), value.at("y").get<double>(),
    value.at("yaw").get<double>()};
}

ReplayData loadSnapshot(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open snapshot: " + path);
  }
  json snapshot;
  input >> snapshot;
  if (snapshot.value("schema", 0) != 1) {
    throw std::runtime_error("unsupported planner replay snapshot schema");
  }

  const auto & map = snapshot.at("map");
  const auto & costmap = snapshot.at("costmap");
  ReplayData result{
    costmap.at("width").get<unsigned int>(),
    costmap.at("height").get<unsigned int>(),
    costmap.at("resolution").get<double>(),
    costmap.at("origin").at("x").get<double>(),
    costmap.at("origin").at("y").get<double>(),
    costmap.at("frame_id").get<std::string>(),
    {},
    {},
    readPose(snapshot.at("start"), "start"),
    readPose(snapshot.at("goal"), "goal"),
    snapshot.at("plan")};

  if (result.width == 0 || result.height == 0 || result.resolution <= 0.0) {
    throw std::runtime_error("snapshot has invalid costmap geometry");
  }
  if (map.at("frame_id").get<std::string>() != result.frame_id ||
    map.at("width").get<unsigned int>() != result.width ||
    map.at("height").get<unsigned int>() != result.height)
  {
    throw std::runtime_error("map and costmap geometry/frame do not agree");
  }
  if (std::abs(costmap.at("origin").at("yaw").get<double>()) > 1.0e-6) {
    throw std::runtime_error("rotated costmap origins are not supported by replay");
  }

  const auto & costs = costmap.at("data");
  if (!costs.is_array() || costs.size() != result.width * result.height) {
    throw std::runtime_error("costmap data length does not match geometry");
  }
  result.costs.reserve(costs.size());
  for (const auto & value : costs) {
    const int cost = value.get<int>();
    if (cost < 0 || cost > 255) {
      throw std::runtime_error("costmap contains a value outside 0..255");
    }
    result.costs.push_back(static_cast<unsigned char>(cost));
  }

  const auto & footprint = snapshot.at("footprint");
  if (!footprint.is_array() || footprint.size() < 3) {
    throw std::runtime_error("snapshot footprint is not a polygon");
  }
  for (const auto & point : footprint) {
    geometry_msgs::msg::Point converted;
    converted.x = point.at(0).get<double>();
    converted.y = point.at(1).get<double>();
    converted.z = 0.0;
    result.footprint.push_back(converted);
  }
  if (result.recorded_plan.at("poses").size() < 3) {
    throw std::runtime_error("recorded plan must contain at least three poses");
  }
  return result;
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> makeCostmap(const ReplayData & data)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    data.width, data.height, data.resolution, data.origin_x, data.origin_y,
    nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int y = 0; y < data.height; ++y) {
    for (unsigned int x = 0; x < data.width; ++x) {
      costmap->setCost(x, y, data.costs[x + y * data.width]);
    }
  }
  return costmap;
}

void mutateEndpoint(
  const ReplayData & data, std::vector<unsigned char> & costs, const std::string & endpoint,
  const unsigned char value)
{
  if (endpoint == "none") {
    return;
  }
  const PoseData & pose = endpoint == "start" ? data.start : data.goal;
  auto costmap = makeCostmap(data);
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(pose.x, pose.y, mx, my)) {
    throw std::runtime_error("endpoint is outside the replay costmap: " + endpoint);
  }
  const int center_x = static_cast<int>(mx);
  const int center_y = static_cast<int>(my);
  for (int dy = -kEndpointBlockRadiusCells; dy <= kEndpointBlockRadiusCells; ++dy) {
    for (int dx = -kEndpointBlockRadiusCells; dx <= kEndpointBlockRadiusCells; ++dx) {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (x >= 0 && y >= 0 && static_cast<unsigned int>(x) < data.width &&
        static_cast<unsigned int>(y) < data.height)
      {
        costs[static_cast<unsigned int>(x) + static_cast<unsigned int>(y) * data.width] =
          value;
      }
    }
  }
}

std::shared_ptr<nav2_costmap_2d::Costmap2D> makeScenarioCostmap(
  const ReplayData & data, const RunOptions & options)
{
  ReplayData scenario = data;
  scenario.costs = data.costs;
  mutateEndpoint(scenario, scenario.costs, options.block_endpoint,
    nav2_costmap_2d::LETHAL_OBSTACLE);
  mutateEndpoint(scenario, scenario.costs, options.unknown_endpoint,
    nav2_costmap_2d::NO_INFORMATION);
  return makeCostmap(scenario);
}

SearchInfo makeSearchInfo()
{
  SearchInfo info{};
  info.minimum_turning_radius = 0.5F;
  info.non_straight_penalty = 1.05F;
  info.change_penalty = 0.05F;
  info.reverse_penalty = 2.0F;
  info.cost_penalty = 2.0F;
  info.retrospective_penalty = 0.015F;
  info.rotation_penalty = 5.0F;
  info.analytic_expansion_ratio = 3.5F;
  info.analytic_expansion_max_length = 3.0F;
  info.cache_obstacle_heuristic = false;
  info.allow_reverse_expansion = false;
  return info;
}

std::shared_ptr<nav2_util::LifecycleNode> makeNode()
{
  return std::make_shared<nav2_util::LifecycleNode>("planner_replay");
}

json poseJson(double x, double y, double yaw)
{
  return json{{"x", x}, {"y", y}, {"yaw", wrapAngle(yaw)}};
}

double distance(const PoseData & first, const PoseData & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

struct CollisionSummary
{
  bool collision_free = true;
  std::size_t first_collision_index = std::numeric_limits<std::size_t>::max();
  double worst_cost = 0.0;
};

CollisionSummary checkWorldPath(
  const ReplayData & data, nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<PoseData> & path, const bool /*allow_unknown*/)
{
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  CollisionSummary summary;
  double circumscribed_radius = 0.0;
  for (const auto & point : data.footprint) {
    circumscribed_radius = std::max(
      circumscribed_radius, std::hypot(point.x, point.y));
  }

  auto check_pose = [&](const PoseData & pose, const std::size_t path_index) {
    const double cost = checker.footprintCostAtPose(
      pose.x, pose.y, pose.yaw, data.footprint);
    summary.worst_cost = std::max(summary.worst_cost, cost);
    // Match CostmapTopicCollisionChecker::isCollisionFree used by the
    // smoother: footprint cost 253 is allowed; 254 and 255 are rejected.
    // Planner allow_unknown does not override the downstream safety gate.
    const bool collision = cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
    if (collision && summary.collision_free) {
      summary.collision_free = false;
      summary.first_collision_index = path_index;
    }
  };

  if (path.empty()) {
    return summary;
  }
  check_pose(path.front(), 0);
  for (std::size_t index = 1; index < path.size(); ++index) {
    const auto & previous = path[index - 1];
    const auto & current = path[index];
    const double translation = std::hypot(
      current.x - previous.x, current.y - previous.y);
    const double rotation_sweep = circumscribed_radius * std::abs(
      wrapAngle(current.yaw - previous.yaw));
    const auto steps = static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        (translation + rotation_sweep) /
        (kFootprintSampleSpacingFraction * data.resolution))));
    for (std::size_t step = 1; step <= steps; ++step) {
      const double fraction = static_cast<double>(step) / static_cast<double>(steps);
      check_pose(PoseData{
          previous.x + fraction * (current.x - previous.x),
          previous.y + fraction * (current.y - previous.y),
          wrapAngle(previous.yaw + fraction * wrapAngle(current.yaw - previous.yaw))},
        index);
    }
  }
  return summary;
}

json collisionJson(const CollisionSummary & summary)
{
  return json{
    {"collision_free", summary.collision_free},
    {"first_collision_index", summary.first_collision_index == std::numeric_limits<std::size_t>::max() ?
        -1 : static_cast<long long>(summary.first_collision_index)},
    {"worst_cost", summary.worst_cost}};
}

std::vector<PoseData> recordedPath(const json & path)
{
  std::vector<PoseData> result;
  for (const auto & pose : path.at("poses")) {
    result.push_back(readPose(pose, "recorded path pose"));
  }
  return result;
}

void addCommonResult(
  json & result, const ReplayData & data, const std::vector<PoseData> & path,
  const CollisionSummary & collision, const int iterations)
{
  result["iterations"] = iterations;
  result["pose_count"] = path.size();
  result["collision"] = collisionJson(collision);
  result["start_error_m"] = path.empty() ? -1.0 : distance(path.front(), data.start);
  result["goal_error_m"] = path.empty() ? -1.0 : distance(path.back(), data.goal);
  result["goal_yaw_error_rad"] = path.empty() ? -1.0 :
    std::abs(wrapAngle(path.back().yaw - data.goal.yaw));
  json poses = json::array();
  for (const auto & pose : path) {
    poses.push_back(poseJson(pose.x, pose.y, pose.yaw));
  }
  result["path"] = std::move(poses);
  result["endpoint_compatible"] = !path.empty() &&
    result["start_error_m"].get<double>() <= data.resolution + 1.0e-6 &&
    result["goal_error_m"].get<double>() <= kGoalPositionTolerance + data.resolution;
  result["endpoint_tolerance_compatible"] = result["endpoint_compatible"];
  result["controller_execution_tested"] = false;
  result["compatibility_scope"] = "endpoint_tolerance_only";
  result["collision_free"] = collision.collision_free;
  result["success"] = result["planner_success"].get<bool>() &&
    result["collision_free"].get<bool>() && result["endpoint_compatible"].get<bool>();
}

#ifndef AMR_REPLAY_LATTICE_ONLY
std::vector<PoseData> convert2DPath(
  const ReplayData & data, nav2_costmap_2d::Costmap2D * costmap,
  const nav2_smac_planner::Node2D::CoordinateVector & path)
{
  std::vector<PoseData> result;
  result.reserve(path.size());
  for (auto iterator = path.rbegin(); iterator != path.rend(); ++iterator) {
    const double x_cell = iterator->x;
    const double y_cell = iterator->y;
    double x = 0.0;
    double y = 0.0;
    costmap->mapToWorld(
      static_cast<unsigned int>(std::lround(x_cell)),
      static_cast<unsigned int>(std::lround(y_cell)), x, y);
    double yaw = data.goal.yaw;
    if (iterator + 1 != path.rend()) {
      double next_x = 0.0;
      double next_y = 0.0;
      costmap->mapToWorld(
        static_cast<unsigned int>(std::lround((iterator + 1)->x)),
        static_cast<unsigned int>(std::lround((iterator + 1)->y)), next_x, next_y);
      yaw = std::atan2(next_y - y, next_x - x);
    } else if (!result.empty()) {
      yaw = result.back().yaw;
    }
    result.push_back(PoseData{x, y, yaw});
  }
  return result;
}
#endif

#ifndef AMR_REPLAY_2D_ONLY
std::vector<PoseData> convertLatticePath(
  const ReplayData & data, nav2_costmap_2d::Costmap2D * costmap,
  const nav2_smac_planner::NodeLattice::CoordinateVector & path)
{
  std::vector<PoseData> result;
  result.reserve(path.size());
  // AStarAlgorithm backtraces from the goal.  The runtime plugin reverses
  // that vector before publishing, so replay must use the same start-to-goal
  // order for endpoint and collision checks.
  for (auto iterator = path.rbegin(); iterator != path.rend(); ++iterator) {
    const auto & coordinate = *iterator;
    // Backtraced lattice poses contain fractional cells and radians, not
    // discrete search heading bins. Match SmacPlannerLattice::createPlan.
    const auto pose = nav2_smac_planner::getWorldCoords(
      coordinate.x, coordinate.y, costmap);
    result.push_back(PoseData{
      pose.position.x, pose.position.y, coordinate.theta});
  }
  return result;
}
#endif

json runConfiguredSmoother(
  const ReplayData & data, const RunOptions & options,
  const std::shared_ptr<nav2_util::LifecycleNode> & node,
  const std::vector<PoseData> & input_path, const std::string & input_label);

#ifndef AMR_REPLAY_LATTICE_ONLY
json runSmac2D(
  const ReplayData & data, const RunOptions & options, const std::shared_ptr<nav2_util::LifecycleNode> & node)
{
  json result{{"planner", "SmacPlanner2D"}, {"planner_success", false}};
  auto costmap = makeScenarioCostmap(data, options);
  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap->worldToMap(data.start.x, data.start.y, start_x, start_y) ||
    !costmap->worldToMap(data.goal.x, data.goal.y, goal_x, goal_y))
  {
    result["error"] = "start or goal is outside costmap";
    return result;
  }

  SearchInfo info = makeSearchInfo();
  unsigned int width = data.width;
  unsigned int height = data.height;
  unsigned int angle_bins = 1;
  nav2_smac_planner::Node2D::initMotionModel(
    MotionModel::TWOD, width, height, angle_bins, info);
  nav2_smac_planner::Node2D::cost_travel_multiplier = 2.0F;
  GridCollisionChecker checker(costmap.get(), 1, node);
  // A zero possible-inscribed cost forces the installed checker to evaluate
  // the supplied polygon instead of taking its optimized center-cost path.
  checker.setFootprint(data.footprint, false, 0.0);
  result["start_cell"] = {start_x, start_y};
  result["goal_cell"] = {goal_x, goal_y};
  result["start_in_collision"] = checker.inCollision(
    static_cast<float>(start_x), static_cast<float>(start_y), 0.0F, options.allow_unknown);
  result["goal_in_collision"] = checker.inCollision(
    static_cast<float>(goal_x), static_cast<float>(goal_y), 0.0F, options.allow_unknown);

  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> planner(
    MotionModel::TWOD, info);
  // The runtime config uses -1 to mean unbounded; the plugin normalizes it
  // to INT_MAX before passing it to AStarAlgorithm::initialize().
  int max_iterations = std::numeric_limits<int>::max();
  planner.initialize(
    options.allow_unknown, max_iterations, kMaxOnApproachIterations,
    kPathPlanningTime, kLookupTableSize, 1);
  planner.setCollisionChecker(&checker);
  planner.setStart(start_x, start_y, 0);
  planner.setGoal(goal_x, goal_y, 0);
  nav2_smac_planner::Node2D::CoordinateVector path;
  int iterations = 0;
  const bool planner_success = planner.createPath(
    path, iterations, static_cast<float>(0.05 / data.resolution));
  result["planner_success"] = planner_success;
  if (!planner_success) {
    result["iterations"] = iterations;
    result["error"] = "SmacPlanner2D createPath returned false";
    result["success"] = false;
    return result;
  }
  const auto world_path = convert2DPath(data, costmap.get(), path);
  const auto collision = checkWorldPath(data, costmap.get(), world_path, options.allow_unknown);
  addCommonResult(result, data, world_path, collision, iterations);
  if (options.run_smoother) {
    result["candidate_smoother"] = runConfiguredSmoother(
      data, options, node, world_path, "smac_2d_path");
    result["success"] = result["success"].get<bool>() &&
      result["candidate_smoother"].value("success", false);
  }
  return result;
}
#endif

#ifndef AMR_REPLAY_2D_ONLY
json runSmacLattice(
  const ReplayData & data, const RunOptions & options,
  const std::shared_ptr<nav2_util::LifecycleNode> & node)
{
  json result{{"planner", "SmacPlannerLattice"}, {"planner_success", false}};
  if (options.lattice_file.empty()) {
    result["error"] = "--lattice is required for SmacPlannerLattice";
    return result;
  }
  auto costmap = makeScenarioCostmap(data, options);
  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap->worldToMap(data.start.x, data.start.y, start_x, start_y) ||
    !costmap->worldToMap(data.goal.x, data.goal.y, goal_x, goal_y))
  {
    result["error"] = "start or goal is outside costmap";
    return result;
  }

  SearchInfo info = makeSearchInfo();
  info.lattice_filepath = options.lattice_file;
  const auto metadata = nav2_smac_planner::LatticeMotionTable::getLatticeMetadata(
    options.lattice_file);
  info.minimum_turning_radius = metadata.min_turning_radius / data.resolution;
  info.analytic_expansion_max_length =
    kLatticeAnalyticExpansionMaxLengthMeters / data.resolution;
  unsigned int width = data.width;
  unsigned int height = data.height;
  unsigned int angle_bins = metadata.number_of_headings;
  nav2_smac_planner::NodeLattice::initMotionModel(
    MotionModel::STATE_LATTICE, width, height, angle_bins, info);
  float lookup_table_dim = static_cast<float>(
    static_cast<int>(kLatticeLookupTableMeters / data.resolution));
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    lookup_table_dim += 1.0F;
  }
  nav2_smac_planner::NodeLattice::precomputeDistanceHeuristic(
    lookup_table_dim, MotionModel::STATE_LATTICE, angle_bins, info);
  GridCollisionChecker checker(costmap.get(), kLatticeCollisionBins, node);
  checker.setFootprint(data.footprint, false, 0.0);

  const unsigned int start_bin = nav2_smac_planner::NodeLattice::motion_table.getClosestAngularBin(
    data.start.yaw);
  const unsigned int goal_bin = nav2_smac_planner::NodeLattice::motion_table.getClosestAngularBin(
    data.goal.yaw);
  nav2_smac_planner::NodeLattice::resetObstacleHeuristic(
    costmap.get(), start_x, start_y, goal_x, goal_y);
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice> planner(
    MotionModel::STATE_LATTICE, info);
  // Keep the replay equivalent to the runtime's non-positive max_iterations
  // normalization instead of making the A* loop exit before its first step.
  int max_iterations = std::numeric_limits<int>::max();
  planner.initialize(
    options.allow_unknown, max_iterations, kMaxOnApproachIterations,
    kPathPlanningTime, lookup_table_dim, angle_bins);
  planner.setCollisionChecker(&checker);
  planner.setStart(start_x, start_y, start_bin);
  planner.setGoal(goal_x, goal_y, goal_bin);
  nav2_smac_planner::NodeLattice::CoordinateVector path;
  int iterations = 0;
  const bool planner_success = planner.createPath(
    path, iterations, static_cast<float>(0.05 / data.resolution));
  result["planner_success"] = planner_success;
  result["heading_bins"] = angle_bins;
  result["analytic_expansion_max_length_m"] = kLatticeAnalyticExpansionMaxLengthMeters;
  result["lattice_min_turning_radius_m"] = metadata.min_turning_radius;
  if (!planner_success) {
    result["iterations"] = iterations;
    result["error"] = "SmacPlannerLattice createPath returned false";
    result["success"] = false;
    return result;
  }
  const auto world_path = convertLatticePath(data, costmap.get(), path);
  const auto collision = checkWorldPath(data, costmap.get(), world_path, options.allow_unknown);
  addCommonResult(result, data, world_path, collision, iterations);
  result["endpoint_tolerance_compatible"] = result["endpoint_compatible"].get<bool>() &&
    result["goal_yaw_error_rad"].get<double>() <= kGoalYawTolerance;
  // Deprecated report alias. This is endpoint compatibility only; this
  // standalone harness does not execute the RPP controller.
  result["controller_compatible"] = result["endpoint_tolerance_compatible"];
  result["success"] = result["success"].get<bool>() &&
    result["endpoint_tolerance_compatible"].get<bool>();
  if (options.run_smoother) {
    result["candidate_smoother"] = runConfiguredSmoother(
      data, options, node, world_path, "smac_lattice_path");
    result["success"] = result["success"].get<bool>() &&
      result["candidate_smoother"].value("success", false);
  }
  return result;
}
#endif

json runConfiguredSmoother(
  const ReplayData & data, const RunOptions & options,
  const std::shared_ptr<nav2_util::LifecycleNode> & node,
  const std::vector<PoseData> & input_path, const std::string & input_label)
{
  json result{
    {"smoother", "nav2_smoother::SimpleSmoother"}, {"input", input_label}, {"success", false}};
  try {
    node->declare_parameter("simple_smoother.tolerance", 1.0e-10);
    node->declare_parameter("simple_smoother.max_its", 1000);
    node->declare_parameter("simple_smoother.w_data", 0.2);
    node->declare_parameter("simple_smoother.w_smooth", 0.0);
    node->declare_parameter("simple_smoother.do_refinement", true);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // The harness is single-use, but keep this safe if the node options change.
  }

  auto costmap_sub = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
    node, "global_costmap/costmap_raw");
  auto message = std::make_shared<nav2_msgs::msg::Costmap>();
  message->header.frame_id = data.frame_id;
  message->metadata.resolution = static_cast<float>(data.resolution);
  message->metadata.size_x = data.width;
  message->metadata.size_y = data.height;
  message->metadata.origin.position.x = data.origin_x;
  message->metadata.origin.position.y = data.origin_y;
  message->metadata.origin.orientation.w = 1.0;
  message->data = data.costs;
  costmap_sub->costmapCallback(message);

  nav_msgs::msg::Path path;
  path.header.frame_id = data.recorded_plan.at("frame_id").get<std::string>();
  for (const auto & pose : input_path) {
    geometry_msgs::msg::PoseStamped converted;
    converted.header.frame_id = path.header.frame_id;
    converted.pose.position.x = pose.x;
    converted.pose.position.y = pose.y;
    converted.pose.orientation.z = std::sin(pose.yaw * 0.5);
    converted.pose.orientation.w = std::cos(pose.yaw * 0.5);
    path.poses.push_back(converted);
  }

  nav2_smoother::SimpleSmoother smoother;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  smoother.configure(node, "simple_smoother", tf_buffer, costmap_sub, nullptr);
  smoother.activate();
  const bool completed = smoother.smooth(path, rclcpp::Duration(std::chrono::seconds(2)));
  smoother.deactivate();
  smoother.cleanup();

  std::vector<PoseData> smoothed_path;
  smoothed_path.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const double yaw = std::atan2(
      2.0 * pose.pose.orientation.w * pose.pose.orientation.z,
      1.0 - 2.0 * pose.pose.orientation.z * pose.pose.orientation.z);
    smoothed_path.push_back(PoseData{pose.pose.position.x, pose.pose.position.y, yaw});
  }
  auto costmap = makeCostmap(data);
  const auto collision = checkWorldPath(data, costmap.get(), smoothed_path, options.allow_unknown);
  result["smoother_completed"] = completed;
  result["pose_count"] = smoothed_path.size();
  result["collision"] = collisionJson(collision);
  result["collision_free"] = collision.collision_free;
  result["start_error_m"] = smoothed_path.empty() ? -1.0 : distance(smoothed_path.front(), data.start);
  result["goal_error_m"] = smoothed_path.empty() ? -1.0 : distance(smoothed_path.back(), data.goal);
  result["goal_yaw_error_rad"] = smoothed_path.empty() ? -1.0 :
    std::abs(wrapAngle(smoothed_path.back().yaw - data.goal.yaw));
  result["endpoint_tolerance_compatible"] = !smoothed_path.empty() &&
    result["goal_error_m"].get<double>() <= kGoalPositionTolerance &&
    result["goal_yaw_error_rad"].get<double>() <= kGoalYawTolerance;
  result["controller_execution_tested"] = false;
  result["compatibility_scope"] = "endpoint_tolerance_only";
  // Deprecated report alias. This is endpoint compatibility only; this
  // standalone harness does not execute the RPP controller.
  result["controller_compatible"] = result["endpoint_tolerance_compatible"];
  result["success"] = completed && collision.collision_free &&
    result["endpoint_tolerance_compatible"].get<bool>();
  return result;
}

std::string requireOption(int argc, char ** argv, const std::string & name)
{
  for (int index = 1; index < argc; ++index) {
    if (argv[index] == name && index + 1 < argc) {
      return argv[index + 1];
    }
  }
  throw std::runtime_error("missing required option " + name);
}

RunOptions parseOptions(int argc, char ** argv)
{
  RunOptions options;
  options.snapshot = requireOption(argc, argv, "--snapshot");
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    auto next = [&](const std::string & name) {
        if (index + 1 >= argc) {
          throw std::runtime_error("missing value for " + name);
        }
        return std::string(argv[++index]);
      };
    if (arg == "--lattice") {
      options.lattice_file = next(arg);
    } else if (arg == "--planner") {
      options.planner = next(arg);
    } else if (arg == "--block-endpoint") {
      options.block_endpoint = next(arg);
    } else if (arg == "--unknown-endpoint") {
      options.unknown_endpoint = next(arg);
    } else if (arg == "--allow-unknown") {
      const std::string value = next(arg);
      options.allow_unknown = value != "false" && value != "0";
    } else if (arg == "--run-smoother") {
      options.run_smoother = true;
    } else if (arg == "--output") {
      options.output = next(arg);
    } else if (arg == "--snapshot") {
      ++index;
    } else if (arg == "--help") {
      throw std::runtime_error(
        "usage: planner_replay_backend --snapshot SNAPSHOT [--lattice FILE] "
        "[--planner all|2d|lattice] [--allow-unknown true|false] "
        "[--block-endpoint none|start|goal] [--unknown-endpoint none|start|goal] "
        "[--run-smoother] [--output FILE]");
    }
  }
  if (options.planner != "all" && options.planner != "2d" && options.planner != "lattice") {
    throw std::runtime_error("--planner must be all, 2d, or lattice");
  }
  if (options.block_endpoint != "none" && options.block_endpoint != "start" &&
    options.block_endpoint != "goal")
  {
    throw std::runtime_error("--block-endpoint must be none, start, or goal");
  }
  if (options.unknown_endpoint != "none" && options.unknown_endpoint != "start" &&
    options.unknown_endpoint != "goal")
  {
    throw std::runtime_error("--unknown-endpoint must be none, start, or goal");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const RunOptions options = parseOptions(argc, argv);
    const ReplayData data = loadSnapshot(options.snapshot);
    rclcpp::init(0, nullptr);
    auto node = makeNode();
    json report{
      {"schema", 1},
      {"snapshot", options.snapshot},
      {"allow_unknown", options.allow_unknown},
      {"block_endpoint", options.block_endpoint},
      {"unknown_endpoint", options.unknown_endpoint},
      {"footprint", json::array()}};
    for (const auto & point : data.footprint) {
      report["footprint"].push_back({point.x, point.y});
    }

    if (options.planner == "all" || options.planner == "2d") {
#ifndef AMR_REPLAY_LATTICE_ONLY
      try {
        report["smac_2d"] = runSmac2D(data, options, node);
      } catch (const std::exception & error) {
        report["smac_2d"] = json{{"planner", "SmacPlanner2D"}, {"success", false}, {"error", error.what()}};
      }
#endif
    }
    if (options.planner == "all" || options.planner == "lattice") {
#ifndef AMR_REPLAY_2D_ONLY
      try {
        report["smac_lattice"] = runSmacLattice(data, options, node);
      } catch (const std::exception & error) {
        report["smac_lattice"] = json{{"planner", "SmacPlannerLattice"}, {"success", false}, {"error", error.what()}};
      }
#endif
    }
    if (options.run_smoother) {
      try {
        report["simple_smoother"] = runConfiguredSmoother(
          data, options, node, recordedPath(data.recorded_plan), "recorded_plan");
      } catch (const std::exception & error) {
        report["simple_smoother"] = json{
          {"smoother", "nav2_smoother::SimpleSmoother"}, {"success", false},
          {"error", error.what()}};
      }
    }

    bool overall = true;
    // The recorded failing path is a diagnostic control, not a candidate.
    // Each selected candidate already includes its own smoother verdict.
    for (const auto & name : {"smac_2d", "smac_lattice"}) {
      if (report.contains(name)) {
        overall = overall && report.at(name).value("success", false);
      }
    }
    report["success"] = overall;
    if (!options.output.empty()) {
      std::ofstream output(options.output);
      if (!output) {
        throw std::runtime_error("cannot write report: " + options.output);
      }
      output << std::setw(2) << report << "\n";
    } else {
      std::cout << std::setw(2) << report << "\n";
    }
    node.reset();
    rclcpp::shutdown();
    return overall ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "planner replay failed: " << error.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
