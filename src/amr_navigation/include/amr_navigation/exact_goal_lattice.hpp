#ifndef AMR_NAVIGATION__EXACT_GOAL_LATTICE_HPP_
#define AMR_NAVIGATION__EXACT_GOAL_LATTICE_HPP_

#include <mutex>

#include "nav2_smac_planner/smac_planner_lattice.hpp"

namespace amr_navigation
{

// Empty means no safe, bounded forward-only connection to the exact goal.
nav_msgs::msg::Path finish_exact_goal_path(
  nav_msgs::msg::Path path, const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap, const nav2_costmap_2d::Footprint & footprint);

class ExactGoalLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Serializes wrapper lifecycle/plan access without holding upstream's
  // nonrecursive mutex when calling its planning implementation.
  std::mutex operation_mutex_;
  bool configured_{false};
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__EXACT_GOAL_LATTICE_HPP_
