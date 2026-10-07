#pragma once

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace amr_mpc_controller
{
// Preserve upstream RPP tracking, but finish large initial turns before translating.
class HeadingLatchedRPP
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * goal_checker) override;

private:
  enum class Phase {NORMAL, STOPPING, ROTATING, SETTLING};
  Phase phase_{Phase::NORMAL};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr contract_callback_;
};
}  // namespace amr_mpc_controller
