#include "amr_navigation/exact_goal_lattice.hpp"

#include <cmath>
#include <iterator>
#include <utility>

#include "amr_navigation/precision_navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{
namespace
{

bool finite_planar_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return !pose.header.frame_id.empty() &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::abs(p.z) <= 1e-6 &&
         std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         std::abs(q.x) <= 1e-6 && std::abs(q.y) <= 1e-6 &&
         std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-6;
}

}  // namespace

nav_msgs::msg::Path finish_exact_goal_path(
  nav_msgs::msg::Path path, const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap, const nav2_costmap_2d::Footprint & footprint)
{
  if (path.poses.empty() || path.header.frame_id != goal.header.frame_id ||
    !finite_planar_pose(goal)) return nav_msgs::msg::Path{};
  for (const auto & pose : path.poses) {
    if (pose.header.frame_id != path.header.frame_id || !finite_planar_pose(pose)) return nav_msgs::msg::Path{};
  }
  const auto & end = path.poses.back();
  const double resolution = costmap.getResolution();
  // Existing 50 mm lattice tolerance plus the largest unavoidable XY offset
  // between a requested point and its 2D cell center (half-cell diagonal).
  const double bound = 0.05 + std::hypot(0.5*resolution, 0.5*resolution);
  const double gap = std::hypot(goal.pose.position.x - end.pose.position.x,
    goal.pose.position.y - end.pose.position.y);
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(bound) || !std::isfinite(gap) || gap > bound) return nav_msgs::msg::Path{};
  auto suffix = make_forward_precision_segment(end, goal, costmap, footprint);
  if (suffix.poses.empty()) return nav_msgs::msg::Path{};
  // The upstream endpoint stays intact. Every appended pose belongs to the
  // collision-checked translation/turn sweep, including the exact requested goal.
  path.poses.insert(path.poses.end(), std::next(suffix.poses.begin()), suffix.poses.end());
  return path;
}

void ExactGoalLattice::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> operation(operation_mutex_);
  SmacPlannerLattice::configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
  configured_ = true;
}

void ExactGoalLattice::cleanup()
{
  std::lock_guard<std::mutex> operation(operation_mutex_);
  configured_ = false;
  SmacPlannerLattice::cleanup();
}

void ExactGoalLattice::activate()
{
  std::lock_guard<std::mutex> operation(operation_mutex_);
  SmacPlannerLattice::activate();
}

void ExactGoalLattice::deactivate()
{
  std::lock_guard<std::mutex> operation(operation_mutex_);
  SmacPlannerLattice::deactivate();
}

nav_msgs::msg::Path ExactGoalLattice::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> operation(operation_mutex_);
  if (!configured_ || !finite_planar_pose(start) || !finite_planar_pose(goal) ||
    start.header.frame_id != _global_frame || goal.header.frame_id != _global_frame) return nav_msgs::msg::Path{};
  auto path = SmacPlannerLattice::createPlan(start, goal);
  // Match upstream's planner->costmap lock order. Its dynamic parameter
  // callback cannot change the contract while the suffix is being checked.
  std::lock_guard<std::mutex> planner(_mutex);
  if (!_costmap || !_costmap_ros || _allow_unknown ||
    std::abs(static_cast<double>(_tolerance) - 0.05) > 1e-6 ||
    _search_info.allow_reverse_expansion || _search_info.analytic_expansion_max_length != 0.0)
  {
    return nav_msgs::msg::Path{};
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map(*_costmap->getMutex());
  return finish_exact_goal_path(std::move(path), goal, *_costmap, _costmap_ros->getRobotFootprint());
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::ExactGoalLattice, nav2_core::GlobalPlanner)
