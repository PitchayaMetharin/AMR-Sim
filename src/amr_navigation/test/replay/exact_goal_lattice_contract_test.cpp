#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "amr_navigation/exact_goal_lattice.hpp"
#include "amr_navigation/precision_navfn_planner.hpp"

namespace {

void require(bool condition, const std::string & reason)
{
  if (!condition) throw std::runtime_error(reason);
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return result;
}

void parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  const std::string primitives =
    "/opt/ros/humble/share/nav2_smac_planner/sample_primitives/"
    "5cm_resolution/0.5m_turning_radius/diff/output.json";
  node->declare_parameter(name + ".lattice_filepath", primitives);
  node->declare_parameter(name + ".tolerance", 0.05);
  node->declare_parameter(name + ".allow_unknown", false);
  node->declare_parameter(name + ".max_iterations", -1);
  node->declare_parameter(name + ".max_on_approach_iterations", 1000);
  node->declare_parameter(name + ".max_planning_time", 2.0);
  node->declare_parameter(name + ".analytic_expansion_max_length", 0.0);
  node->declare_parameter(name + ".analytic_expansion_ratio", 3.5);
  node->declare_parameter(name + ".smooth_path", false);
  node->declare_parameter(name + ".lookup_table_size", 20.0);
  node->declare_parameter(name + ".cost_penalty", 2.0);
  node->declare_parameter(name + ".change_penalty", 0.05);
  node->declare_parameter(name + ".non_straight_penalty", 1.05);
  node->declare_parameter(name + ".reverse_penalty", 2.0);
  node->declare_parameter(name + ".retrospective_penalty", 0.015);
  node->declare_parameter(name + ".rotation_penalty", 5.0);
  node->declare_parameter(name + ".allow_reverse_expansion", false);
  node->declare_parameter(name + ".cache_obstacle_heuristic", false);
}

}  // namespace

int main()
{
  rclcpp::init(0, nullptr);
  try {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("exact_lattice_test");
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("exact_lattice_costmap");
    costmap->set_parameters({rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base_footprint"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("width", 12), rclcpp::Parameter("height", 10),
      rclcpp::Parameter("origin_x", -6.0), rclcpp::Parameter("origin_y", -5.0),
      rclcpp::Parameter("resolution", 0.05),
      rclcpp::Parameter("footprint", "[[0.6,0.4],[0.6,-0.4],[-0.6,-0.4],[-0.6,0.4]]"),
      rclcpp::Parameter("footprint_padding", 0.01)});
    require(costmap->on_configure(rclcpp_lifecycle::State()) == nav2_util::CallbackReturn::SUCCESS,
      "costmap fixture configuration failed");
    auto * map = costmap->getCostmap();
    for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) map->setCost(x, y, 0);
    }
    parameters(node, "baseline");
    nav2_smac_planner::SmacPlannerLattice baseline;
    baseline.configure(node, "baseline", tf, costmap);
    baseline.activate();
    // Existing loaded transport: pickup A station to the registered dispatch
    // approach, commanded with its translation bearing rather than dock yaw.
    const auto start = pose(1.5, 3.0, 0.0);
    const auto goal = pose(-2.5, 0.0, std::atan2(-3.0, -4.0));
    const auto snapped = baseline.createPlan(start, goal);
    require(!snapped.poses.empty(), "upstream baseline produced no path");
    const double error = std::hypot(snapped.poses.back().pose.position.x - goal.pose.position.x,
      snapped.poses.back().pose.position.y - goal.pose.position.y);
    const double yaw_error = std::abs(std::remainder(
      tf2::getYaw(snapped.poses.back().pose.orientation) - tf2::getYaw(goal.pose.orientation), 2*M_PI));
    require(error > 1e-6 || yaw_error > 1e-6,
      "NON-DIAGNOSTIC baseline: upstream endpoint was already exact");
    std::cout << "Genuine upstream baseline endpoint error: " << error
              << " m, " << yaw_error << " rad; poses=" << snapped.poses.size() << '\n';
    parameters(node, "candidate");
    amr_navigation::ExactGoalLattice candidate;
    candidate.configure(node, "candidate", tf, costmap);
    candidate.activate();
    const auto exact = candidate.createPlan(start, goal);
    require(exact.poses.size() > snapped.poses.size(), "candidate did not retain prefix and add checked suffix");
    for (std::size_t i = 0; i < snapped.poses.size(); ++i) {
      require(exact.poses[i].pose == snapped.poses[i].pose, "candidate changed an upstream prefix pose");
    }
    require(exact.poses.back().pose == goal.pose, "candidate endpoint is not exactly the requested pose");
    for (std::size_t i = snapped.poses.size(); i < exact.poses.size(); ++i) {
      const auto & before = exact.poses[i-1].pose;
      const auto & after = exact.poses[i].pose;
      const double dx = after.position.x - before.position.x;
      const double dy = after.position.y - before.position.y;
      if (std::hypot(dx, dy) > 1e-12) {
        const double yaw = tf2::getYaw(before.orientation);
        require(std::abs(std::remainder(tf2::getYaw(after.orientation)-yaw, 2*M_PI)) < 1e-12,
          "suffix translated during an endpoint turn");
        require(dx*std::cos(yaw) + dy*std::sin(yaw) > 0.0,
          "suffix translated backward");
        require(std::hypot(dx,dy) <= 0.025 + 1e-12, "suffix translation spacing exceeded half cell");
      }
    }
    const auto footprint = costmap->getRobotFootprint();
    const auto one_pose_path = [](const geometry_msgs::msg::PoseStamped & p) {
        nav_msgs::msg::Path path;
        path.header = p.header;
        path.poses.push_back(p);
        return path;
      };
    auto short_start = pose(0.0, 0.0, 0.0);
    auto short_goal = pose(0.07, 0.0, 0.0);
    auto prefix = one_pose_path(short_start);
    require(!amr_navigation::finish_exact_goal_path(prefix, short_goal, *map, footprint).poses.empty(),
      "clear bounded suffix rejected");
    const auto already_exact = amr_navigation::finish_exact_goal_path(
      one_pose_path(short_goal), short_goal, *map, footprint);
    require(!already_exact.poses.empty() && already_exact.poses.back().pose == short_goal.pose,
      "exact upstream endpoint was changed or rejected");
    require(amr_navigation::finish_exact_goal_path(prefix, pose(0.086,0,0), *map, footprint).poses.empty(),
      "suffix exceeded lattice tolerance plus half-cell diagonal");
    require(amr_navigation::finish_exact_goal_path(nav_msgs::msg::Path{}, short_goal, *map, footprint).poses.empty(),
      "empty upstream path accepted");
    auto invalid_path = prefix;
    invalid_path.poses[0].header.frame_id = "odom";
    require(amr_navigation::finish_exact_goal_path(invalid_path, short_goal, *map, footprint).poses.empty(),
      "wrong path pose frame accepted");
    invalid_path = prefix;
    invalid_path.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
    require(amr_navigation::finish_exact_goal_path(invalid_path, short_goal, *map, footprint).poses.empty(),
      "nonfinite upstream path accepted");
    invalid_path = prefix;
    invalid_path.poses[0].pose.orientation.w = 2.0;
    require(amr_navigation::finish_exact_goal_path(invalid_path, short_goal, *map, footprint).poses.empty(),
      "unnormalized upstream quaternion accepted");
    auto invalid_goal = short_goal;
    invalid_goal.pose.orientation = geometry_msgs::msg::Quaternion{};
    invalid_goal.pose.orientation.w = 0.0;
    require(invalid_goal.pose.orientation.x == 0.0 && invalid_goal.pose.orientation.y == 0.0 &&
      invalid_goal.pose.orientation.z == 0.0 && invalid_goal.pose.orientation.w == 0.0,
      "invalid quaternion fixture must have zero norm");
    require(amr_navigation::finish_exact_goal_path(prefix, invalid_goal, *map, footprint).poses.empty(),
      "invalid goal quaternion accepted");
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
    {
      invalid_goal = short_goal;
      invalid_goal.pose.position.x = invalid;
      require(candidate.createPlan(short_start, invalid_goal).poses.empty(), "nonfinite goal reached delegate");
      invalid_goal = short_goal;
      invalid_goal.pose.orientation.z = invalid;
      require(candidate.createPlan(short_start, invalid_goal).poses.empty(), "nonfinite quaternion reached delegate");
    }
    invalid_goal = short_goal;
    invalid_goal.pose.position.z = 0.1;
    require(candidate.createPlan(short_start, invalid_goal).poses.empty(), "nonplanar goal reached delegate");
    invalid_goal = short_goal;
    invalid_goal.header.frame_id = "odom";
    require(candidate.createPlan(short_start, invalid_goal).poses.empty(), "invalid goal frame reached delegate");
    unsigned int mx, my;
    require(map->worldToMap(short_goal.pose.position.x, short_goal.pose.position.y, mx, my), "blockedgoal outside map");
    for (unsigned char cost : {253, 254, 255}) {
      map->setCost(mx, my, cost);
      require(amr_navigation::finish_exact_goal_path(prefix, short_goal, *map, footprint).poses.empty(),
        "inscribed/lethal/unknown endpoint accepted");
    }
    map->setCost(mx, my, 0);
    // A full rectangular footprint is clear at both terminal orientations,
    // while its intermediate corner sweep reaches this captured fixture cell.
    require(map->worldToMap(0.675,0.175,mx,my), "turn obstruction outside map");
    map->setCost(mx, my, 254);
    require(amr_navigation::finish_exact_goal_path(prefix, pose(-0.05,0,0), *map, footprint).poses.empty(),
      "initial forward-only turn ignored an intermediate footprint collision");
    require(amr_navigation::finish_exact_goal_path(prefix, pose(0.05,0,M_PI_2), *map, footprint).poses.empty(),
      "terminal turn ignored an intermediate footprint collision");
    map->setCost(mx, my, 0);
    // Public reversing behavior stays body-heading-preserving; only the new
    // forward-only helper performs the checked half-turn for a rearward goal.
    const auto reverse_goal = pose(-0.05,0,0);
    const auto reverse = amr_navigation::make_precision_segment(short_start, reverse_goal, *map, footprint);
    require(reverse.poses.size() == 3, "public precision segment sample count changed");
    for (const auto & p : reverse.poses) require(std::abs(tf2::getYaw(p.pose.orientation)) < 1e-12,
        "public reversing helper body heading changed");
    const auto forward = amr_navigation::make_forward_precision_segment(short_start, reverse_goal, *map, footprint);
    require(forward.poses.size() > reverse.poses.size(), "forward-only helper omitted sampled turns");
    // Repeat the bound at a different resolution; it must derive from the map.
    nav2_costmap_2d::Costmap2D fine_map(400,400,0.01,-2,-2,0);
    require(amr_navigation::finish_exact_goal_path(prefix, pose(0.058,0,0), fine_map, footprint).poses.empty(),
      "suffix bound used a hardcoded 5cm resolution");
    const auto fine_goal = pose(0.055,0,0);
    require(!amr_navigation::finish_exact_goal_path(prefix, fine_goal, fine_map, footprint).poses.empty(),
      "fine-map clear suffix rejected");
    require(fine_map.worldToMap(0.025,0,mx,my), "interior suffix obstruction outside map");
    fine_map.setCost(mx,my,253);
    require(amr_navigation::finish_exact_goal_path(prefix, fine_goal, fine_map, footprint).poses.empty(),
      "center-clear endpoints bypassed an inscribed interior suffix cell");
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
    require(static_cast<bool>(loader.createSharedInstance("amr_navigation/ExactGoalLattice")),
      "installed exact lattice plugin load failed");
    std::cout << "Candidate preserves upstream prefix and exact goal; swept forward-only suffix, turns, "
              << "cost/frame/finite/bound negatives and public helper parity PASS\n";
    candidate.deactivate();
    candidate.cleanup();
    require(candidate.createPlan(start,goal).poses.empty(), "cleaned-up planner admitted a plan");
    baseline.deactivate();
    baseline.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "exact lattice contract failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
