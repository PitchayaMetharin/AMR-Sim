#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include "amr_mpc_controller/heading_latched_rpp.hpp"

namespace amr_mpc_controller
{
// Private factory controllers: upstream tracking remains the command authority.
template<class Base>
class ProfiledRPP : public Base
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & limit, const bool & percentage) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker) override;

protected:
  // Fixed-size observation only; all access uses the existing sample mutex.
  enum class ReceiveDecision : uint8_t {None, NotSampling, Invalid, Duplicate, Reordered, Accepted, Pending};
  struct ReceiveSnapshot
  {
    ReceiveDecision decision{ReceiveDecision::None};
    uint64_t sequence{0}, callbacks{0};
    int32_t sec{0};
    uint32_t nsec{0};
    int64_t acquired{0}, now{-1}, previous_clock{-1}, high_water_before{0}, high_water_after{0};
    int64_t steady_ns{0}, receipt_ns{0};
    bool sampling{false}, rollback{false}, eligible_before{false}, eligible_after{false};
    bool bad_frame{false}, bad_pose{false}, bad_sec{false}, bad_nsec{false};
    bool nonpositive{false}, future{false}, stale{false};
  };
  struct FailureSnapshot
  {
    uint64_t sequence{0}, callbacks{0}, generation{0};
    int64_t now{0}, previous_clock{-1}, high_water_before{0}, high_water{0};
    int64_t acquired{0}, steady_ns{0}, receipt_ns{0};
    int32_t sec{0};
    uint32_t nsec{0};
    double receipt_age{0.0};
    std::array<double, 7> cached_pose{};  // x/y/z in m, quaternion x/y/z/w.
    bool configured{false}, active{false}, sampling{false}, eligible{false};
    bool inactive{false}, rollback{false}, ineligible{false}, future{false};
    bool acquisition_stale{false}, receipt_stale{false};
    ReceiveSnapshot receive{};
  };
  ReceiveSnapshot last_receive_{};
  FailureSnapshot last_failure_{};
  uint64_t receive_sequence_{0}, failure_sequence_{0};
  std::mutex profile_mutex_, sample_mutex_;
  bool configured_{false}, active_{false}, sampling_{false}, eligible_{false};
  bool external_valid_{true};
  double external_cap_{0.50}, floor_{0.0};
  int product_{0}, last_band_{-1};
  uint64_t generation_{0}, callbacks_{0};
  int64_t high_water_{0}, last_clock_{-1};
  std::array<double, 3> reference_{};
  geometry_msgs::msg::PoseStamped sample_;
  std::chrono::steady_clock::time_point receipt_, last_log_{};
  enum class PendingState : uint8_t {None, Waiting, Failed};
  PendingState pending_state_{PendingState::None};
  bool pending_allowed_{false};  // sample mutex; lifecycle enables waiting only while active.
  uint64_t sampling_token_{0};
  geometry_msgs::msg::PoseStamped pending_;
  std::chrono::steady_clock::time_point pending_receipt_, pending_episode_start_;

private:
  bool observe_clock(int64_t now);
  void receive(const geometry_msgs::msg::PoseStamped & pose, uint64_t token);
  void install_guard();
  void reset_sample();
  std::vector<rclcpp::Parameter> expected_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};

using UpstreamRPP = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
class FinalPositionRPP : public ProfiledRPP<HeadingLatchedRPP> {};
class FinalPositionPlacementRPP : public ProfiledRPP<UpstreamRPP> {};
extern template class ProfiledRPP<HeadingLatchedRPP>;
extern template class ProfiledRPP<UpstreamRPP>;
}  // namespace amr_mpc_controller
