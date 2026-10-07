#ifndef AMR_NAVIGATION__PRECISION_NAVFN_PLANNER_HPP_
#define AMR_NAVIGATION__PRECISION_NAVFN_PLANNER_HPP_

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"

namespace amr_navigation
{

// Empty means that the exact straight segment (including endpoint turns) is
// unsafe. The caller can then use Navfn's existing obstacle-avoiding route.
nav_msgs::msg::Path make_precision_segment(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav2_costmap_2d::Footprint & footprint);

// Forward-only variant with explicit sampled endpoint turns. Used by the
// private lattice suffix; the public reversing segment contract is unchanged.
nav_msgs::msg::Path make_forward_precision_segment(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D & costmap,
  const nav2_costmap_2d::Footprint & footprint);

class PrecisionNavfnPlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__PRECISION_NAVFN_PLANNER_HPP_
