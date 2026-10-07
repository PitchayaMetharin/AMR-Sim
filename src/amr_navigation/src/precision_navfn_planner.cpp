#include "amr_navigation/precision_navfn_planner.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace amr_navigation
{

static nav_msgs::msg::Path make_precision_segment_impl(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav2_costmap_2d::Footprint & footprint, bool allow_reversing)
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double length = std::hypot(dx, dy);
  const double resolution = costmap.getResolution();
  if (start.header.frame_id != goal.header.frame_id || footprint.size() < 3 ||
    !std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
    !std::isfinite(length) || !std::isfinite(start_yaw) || !std::isfinite(goal_yaw) ||
    !std::isfinite(resolution) || resolution <= 0.0)
  {
    return path;
  }

  double radius = 0.0;
  for (const auto & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return path;
    radius = std::max(radius, std::hypot(point.x, point.y));
  }
  if (!allow_reversing && (!std::isfinite(radius) || radius <= 0.0)) return path;
  const auto angle_delta = [](double from, double to) {
      return std::remainder(to - from, 2.0 * M_PI);
    };
  double heading = start_yaw;
  if (length > 0.0) {
    const double forward = std::atan2(dy, dx);
    const double backward = std::remainder(forward + M_PI, 2.0 * M_PI);
    // PlacementFollowPath permits reversing. Preserve the closest body heading
    // instead of requiring a half-turn beside the docking pedestal.
    heading = !allow_reversing || std::abs(angle_delta(start_yaw, forward)) <=
      std::abs(angle_delta(start_yaw, backward)) ? forward : backward;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(
    &costmap);
  const auto clear = [&](double x, double y, double yaw) {
      unsigned int mx, my;
      if (!costmap.worldToMap(x, y, mx, my) ||
        costmap.getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return false;
      }
      const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
      return cost >= 0.0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
    };
  const auto append = [&](double x, double y, double yaw) {
      if (!clear(x, y, yaw)) return false;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      path.poses.push_back(pose);
      return true;
    };
  // Sample both translation and corner sweep at half a costmap cell, matching
  // the existing planner replay's swept-footprint proof.
  const double spacing = 0.5 * resolution;
  const auto turn = [&](double x, double y, double from, double to, bool emit = false) {
      const double delta = angle_delta(from, to);
      const auto steps = static_cast<std::size_t>(std::max(
        1.0, std::ceil(radius * std::abs(delta) / spacing)));
      for (std::size_t i = 0; i <= steps; ++i) {
        const double yaw = from + delta * static_cast<double>(i) / steps;
        if (emit ? !append(x, y, yaw) : !clear(x, y, yaw)) return false;
      }
      return true;
    };
  if (!turn(start.pose.position.x, start.pose.position.y, start_yaw, heading) ||
    !turn(goal.pose.position.x, goal.pose.position.y, heading, goal_yaw))
  {
    return path;
  }
  if (!allow_reversing &&
    !turn(start.pose.position.x, start.pose.position.y, start_yaw, heading, true))
  {
    path.poses.clear();
    return path;
  }
  const auto steps = static_cast<std::size_t>(std::max(1.0, std::ceil(length / spacing)));
  for (std::size_t i = 0; i <= steps; ++i) {
    const double fraction = static_cast<double>(i) / steps;
    if (!append(
        start.pose.position.x + fraction * dx, start.pose.position.y + fraction * dy, heading))
    {
      path.poses.clear();
      return path;
    }
  }
  if (!allow_reversing &&
    !turn(goal.pose.position.x, goal.pose.position.y, heading, goal_yaw, true))
  {
    path.poses.clear();
    return path;
  }
  // The requested orientation is a separate terminal turn, not the tangent of
  // an artificial grid-to-exact-goal diagonal. Preserve the exact endpoints.
  path.poses.front() = start;
  path.poses.back() = goal;
  return path;
}

nav_msgs::msg::Path make_precision_segment(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav2_costmap_2d::Footprint & footprint)
{
  return make_precision_segment_impl(start, goal, costmap, footprint, true);
}

nav_msgs::msg::Path make_forward_precision_segment(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav2_costmap_2d::Footprint & footprint)
{
  return make_precision_segment_impl(start, goal, costmap, footprint, false);
}

void PrecisionNavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  NavfnPlanner::configure(parent, std::move(name), std::move(tf), costmap_ros);
  costmap_ros_ = std::move(costmap_ros);
}

void PrecisionNavfnPlanner::cleanup()
{
  costmap_ros_.reset();
  NavfnPlanner::cleanup();
}

nav_msgs::msg::Path PrecisionNavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    return nav_msgs::msg::Path{};
  }
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    auto path = make_precision_segment(start, goal, *costmap_, costmap_ros_->getRobotFootprint());
    if (!path.poses.empty()) {
      path.header.stamp = clock_->now();
      for (auto & pose : path.poses) pose.header = path.header;
      return path;
    }
  }
  return NavfnPlanner::createPlan(start, goal);
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::PrecisionNavfnPlanner, nav2_core::GlobalPlanner)
