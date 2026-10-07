#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include "amr_interfaces/final_placement_stance.hpp"
#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "amr_manipulation/attachment_gate.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl/time.h"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace tf2_ros {
class BoundaryReturnCorruptingBuffer : public Buffer {
public:
  using Buffer::Buffer;
  using Buffer::lookupTransform;

  void enable_scaled_return(bool enabled)
  {
    scaled_return_.store(enabled, std::memory_order_relaxed);
  }

  void set_post_lookup_hook(std::function<void()> hook)
  {
    std::lock_guard<std::mutex> lock(post_lookup_hook_mutex_);
    post_lookup_hook_ = std::move(hook);
  }

  geometry_msgs::msg::TransformStamped native_lookup_transform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time) const
  {
    return tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time);
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time) const override
  {
    auto transform = tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time);
    if (target_frame == "map" && source_frame == "base_footprint" &&
      time == tf2::TimePointZero)
    {
      std::function<void()> post_lookup_hook;
      {
        std::lock_guard<std::mutex> lock(post_lookup_hook_mutex_);
        post_lookup_hook = post_lookup_hook_;
      }
      if (post_lookup_hook) {
        post_lookup_hook();
        // A test hook may jump ROS time and refresh the buffer evidence.
        transform = tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time);
      }
      if (scaled_return_.load(std::memory_order_relaxed)) {
        auto & q = transform.transform.rotation;
        q.x *= 1.0001;
        q.y *= 1.0001;
        q.z *= 1.0001;
        q.w *= 1.0001;
      }
    }
    return transform;
  }

private:
  std::atomic_bool scaled_return_{false};
  mutable std::mutex post_lookup_hook_mutex_;
  std::function<void()> post_lookup_hook_;
};
}

#define private public
#define Buffer BoundaryReturnCorruptingBuffer
#define main gate6_mass_stage_test_entrypoint
#include "../src/gate6_mass_stage.cpp"
#undef main
#undef Buffer
#undef private

using namespace std::chrono_literals;

namespace {

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

amr_manipulation::ProductSpec test_product()
{
  amr_manipulation::ProductSpec product{};
  product.id = 101;
  product.model = "product_a";
  product.mass_kg = 1.0;
  product.size = {0.30, 0.30, 0.30};
  product.pickup_station = {1.50, 3.00, 0.0};
  product.pickup_dock = {2.40, 3.00, 0.0};
  product.pickup_egress = {1.90, 3.00, 0.0};
  product.pickup_egress_speed_mps = 0.10;
  product.pickup_egress_time_limit_s = 65.0;
  product.pickup_egress_max_distance_m = 0.50;
  product.dispatch_approach = {0.0, 0.0, 0.0};
  product.dispatch_dock = {0.0, 0.0, 0.0};
  product.dispatch_slots[0] = {0.0, 0.0, 0.0};
  product.dispatch_slots[1] = {0.0, 0.0, 0.0};
  product.dispatch_slots[2] = {0.0, 0.0, 0.0};
  product.selected_slot_index = 0;
  product.status_topic = "/amr/test/manipulation_status_101";
  product.cancel_service = "/amr/test/cancel_101";
  return product;
}

class NavigateActionHarness {
 public:
  using Action = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using GoalUUID = rclcpp_action::GoalUUID;

  enum class Behavior {
    SUCCEED_WITH_FEEDBACK,
    SUCCEED_WITHOUT_FEEDBACK,
    REJECT,
    ABORT_WITH_FEEDBACK,
    CANCELED_WITH_FEEDBACK,
    HOLD_NO_FEEDBACK,
    HOLD_STALE_FEEDBACK,
    HOLD_MALFORMED_FEEDBACK,
  };

  struct GoalObservation {
    GoalUUID uuid{};
    std::array<double, 3> target{};
    std::string frame_id;
    std::chrono::steady_clock::time_point requested_at{};
  };

  class RequestBarrier {
   public:
    void record_request(const GoalUUID & uuid)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        uuid_ = uuid;
        request_seen_ = true;
      }
      condition_.notify_all();
    }

    bool wait_for_request(std::chrono::milliseconds timeout)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      return condition_.wait_for(lock, timeout, [this]() {return request_seen_;});
    }

    GoalUUID requested_uuid() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return uuid_;
    }

    bool wait_until_released_or_shutdown(const std::atomic_bool & shutting_down)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      const bool released = condition_.wait_for(lock, 5s, [this, &shutting_down]() {
          return released_ || shutting_down.load();
        });
      return released && released_;
    }

    void release()
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
      }
      condition_.notify_all();
    }

   private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    GoalUUID uuid_{};
    bool request_seen_{false};
    bool released_{false};
  };

  NavigateActionHarness(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & endpoint,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : endpoint_(endpoint)
  {
    server_ = rclcpp_action::create_server<Action>(
      node, endpoint_,
      [this](
        const GoalUUID & uuid,
        std::shared_ptr<const Action::Goal> goal) {
        const auto requested_at = std::chrono::steady_clock::now();
        std::shared_ptr<RequestBarrier> barrier;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          barrier = request_barrier_;
        }
        if (barrier) {
          barrier->record_request(uuid);
          if (!barrier->wait_until_released_or_shutdown(shutting_down_)) {
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        const auto behavior = behavior_.load();
        if (behavior == Behavior::REJECT) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        GoalObservation observation;
        observation.uuid = uuid;
        observation.target = {
          goal->pose.pose.position.x,
          goal->pose.pose.position.y,
          std::atan2(
          2.0 * goal->pose.pose.orientation.w * goal->pose.pose.orientation.z,
          1.0 - 2.0 * goal->pose.pose.orientation.z * goal->pose.pose.orientation.z)};
        observation.frame_id = goal->pose.header.frame_id;
        observation.requested_at = requested_at;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          goals_.push_back(observation);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle> goal) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          canceled_goals_.push_back(goal->get_goal_id());
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> goal) {
        const auto behavior = behavior_.load();
        std::function<void(const Action::Goal &)> accepted;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          accepted_goals_.push_back(goal->get_goal_id());
          accepted = accepted_hook_;
        }
        if (accepted) accepted(*goal->get_goal());
        const auto publish_feedback = [this, goal, behavior]() {
          auto feedback = std::make_shared<Action::Feedback>();
          feedback->current_pose.header.frame_id =
            behavior == Behavior::HOLD_MALFORMED_FEEDBACK ? "odom" : "map";
          feedback->current_pose.pose = goal->get_goal()->pose.pose;
          feedback->navigation_time.sec = 0;
          feedback->navigation_time.nanosec = 1000000;
          feedback->distance_remaining = 0.0;
          if (behavior == Behavior::HOLD_MALFORMED_FEEDBACK) {
            feedback->current_pose.pose.position.x =
              std::numeric_limits<double>::quiet_NaN();
          }
          std::function<void(Action::Feedback &)> mutation;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            mutation = feedback_mutation_;
          }
          if (mutation) mutation(*feedback);
          goal->publish_feedback(feedback);
        };

        if (behavior == Behavior::SUCCEED_WITH_FEEDBACK ||
          behavior == Behavior::ABORT_WITH_FEEDBACK)
        {
          for (int index = 0; index < 5; ++index) {
            publish_feedback();
            std::this_thread::sleep_for(20ms);
          }
          auto result = std::make_shared<Action::Result>();
          if (behavior == Behavior::SUCCEED_WITH_FEEDBACK) {
            goal->succeed(result);
            std::lock_guard<std::mutex> lock(mutex_);
            terminal_succeeded_goals_.push_back(goal->get_goal_id());
          } else if (behavior == Behavior::ABORT_WITH_FEEDBACK) {
            goal->abort(result);
          }
          return;
        }

        if (behavior == Behavior::SUCCEED_WITHOUT_FEEDBACK) {
          goal->succeed(std::make_shared<Action::Result>());
          std::lock_guard<std::mutex> lock(mutex_);
          terminal_succeeded_goals_.push_back(goal->get_goal_id());
          return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        workers_.emplace_back([this, goal, behavior, publish_feedback]() {
          if (behavior == Behavior::CANCELED_WITH_FEEDBACK ||
            behavior == Behavior::HOLD_STALE_FEEDBACK ||
            behavior == Behavior::HOLD_MALFORMED_FEEDBACK)
          {
            for (int index = 0; index < 5; ++index) {
              if (shutting_down_.load() || goal->is_canceling()) break;
              publish_feedback();
              std::this_thread::sleep_for(20ms);
            }
          }
          while (rclcpp::ok() && !shutting_down_.load() && !goal->is_canceling()) {
            std::this_thread::sleep_for(5ms);
          }
          if (!shutting_down_.load() && goal->is_canceling()) {
            goal->canceled(std::make_shared<Action::Result>());
            std::lock_guard<std::mutex> lock(mutex_);
            terminal_canceled_goals_.push_back(goal->get_goal_id());
          }
        });
      }, rcl_action_server_get_default_options(), callback_group);
  }

  ~NavigateActionHarness()
  {
    release_request_barrier();
    shutting_down_.store(true);
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  void set_behavior(Behavior behavior)
  {
    behavior_.store(behavior);
  }

  void set_accepted_hook(std::function<void(const Action::Goal &)> hook)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    accepted_hook_ = std::move(hook);
  }

  void set_feedback_mutation(std::function<void(Action::Feedback &)> mutation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_mutation_ = std::move(mutation);
  }

  void set_request_barrier(const std::shared_ptr<RequestBarrier> & barrier)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_barrier_ = barrier;
  }

  void release_request_barrier()
  {
    std::shared_ptr<RequestBarrier> barrier;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      barrier = request_barrier_;
    }
    if (barrier) barrier->release();
  }

  std::size_t goal_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.size();
  }

  GoalObservation last_goal() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.back();
  }

  std::size_t canceled_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return canceled_goals_.size();
  }

  bool was_canceled(const GoalUUID & uuid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & canceled : canceled_goals_) {
      if (canceled == uuid) return true;
    }
    return false;
  }

  bool was_accepted(const GoalUUID & uuid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & accepted : accepted_goals_) {
      if (accepted == uuid) return true;
    }
    return false;
  }

  bool was_terminal_canceled(const GoalUUID & uuid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & canceled : terminal_canceled_goals_) {
      if (canceled == uuid) return true;
    }
    return false;
  }

  bool was_terminal_succeeded(const GoalUUID & uuid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & succeeded : terminal_succeeded_goals_) {
      if (succeeded == uuid) return true;
    }
    return false;
  }

 private:
  std::string endpoint_;
  std::atomic<Behavior> behavior_{Behavior::SUCCEED_WITH_FEEDBACK};
  std::atomic_bool shutting_down_{false};
  mutable std::mutex mutex_;
  std::vector<GoalObservation> goals_;
  std::vector<GoalUUID> canceled_goals_;
  std::vector<GoalUUID> accepted_goals_;
  std::vector<GoalUUID> terminal_canceled_goals_;
  std::vector<GoalUUID> terminal_succeeded_goals_;
  std::vector<std::thread> workers_;
  std::shared_ptr<RequestBarrier> request_barrier_;
  std::function<void(const Action::Goal &)> accepted_hook_;
  std::function<void(Action::Feedback &)> feedback_mutation_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

class Gate6PickupRetreatBehavior : public ::testing::Test {
 protected:
  virtual void configure_node_before_executor_start() {}
  virtual bool use_reentrant_centered_callback_group() const {return false;}
  virtual amr_manipulation::ProductSpec product_spec() {return test_product();}
  void SetUp() override
  {
    static std::atomic_uint node_index{0};
    const auto index = node_index.fetch_add(1);
    peer_ = std::make_shared<rclcpp::Node>(
      "gate6_pickup_retreat_peer_" + std::to_string(index));
    centered_callback_group_ = use_reentrant_centered_callback_group() ?
      peer_->create_callback_group(rclcpp::CallbackGroupType::Reentrant) : nullptr;
    amcl_pub_ = peer_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amr/amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    normal_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose");
    retreat_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose_retreat");
    precise_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose_precise");
    for (std::size_t index=0;index<2;++index) {
      const std::string endpoint = index==0 ? "/amr/mission/navigate_to_pose_dispatch_a" :
        "/amr/mission/navigate_to_pose_dispatch_b";
      const auto callback_group = index == 1 ? centered_callback_group_ : nullptr;
      dispatch_servers_[index] = std::make_unique<NavigateActionHarness>(
        peer_, endpoint, callback_group);
      dispatch_precise_servers_[index] = std::make_unique<NavigateActionHarness>(
        peer_, endpoint + "_precise", callback_group);
    }
    clear_approach_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose_dispatch_b_clear_approach",
      centered_callback_group_);
    node_ = std::make_shared<amr_manipulation::MassStageNode>(
      product_spec(), rclcpp::NodeOptions());
    configure_node_before_executor_start();
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(peer_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    ASSERT_TRUE(node_->navigation_client_->wait_for_action_server(2s));
    ASSERT_TRUE(node_->retreat_navigation_client_->wait_for_action_server(2s));
    ASSERT_TRUE(node_->precise_navigation_client_->wait_for_action_server(2s));
    if (node_->dispatch_navigation_client_) {
      ASSERT_TRUE(node_->dispatch_navigation_client_->wait_for_action_server(2s));
      ASSERT_TRUE(node_->dispatch_precise_navigation_client_->wait_for_action_server(2s));
    }
    if (node_->dispatch_b_clear_approach_client_)
      ASSERT_TRUE(node_->dispatch_b_clear_approach_client_->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    if (normal_server_) normal_server_->release_request_barrier();
    if (retreat_server_) retreat_server_->release_request_barrier();
    if (precise_server_) precise_server_->release_request_barrier();
    for (auto & server : dispatch_servers_) {
      if (server) server->release_request_barrier();
    }
    for (auto & server : dispatch_precise_servers_) {
      if (server) server->release_request_barrier();
    }
    if (clear_approach_server_) clear_approach_server_->release_request_barrier();
    if (node_) {
      node_->cancel_requested_.store(true);
    }
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (node_) executor_.remove_node(node_->get_node_base_interface());
    if (peer_) executor_.remove_node(peer_->get_node_base_interface());
    executor_.set_memory_strategy(rclcpp::memory_strategies::create_default_strategy());
    normal_server_.reset();
    retreat_server_.reset();
    precise_server_.reset();
    for (auto & server : dispatch_servers_) server.reset();
    for (auto & server : dispatch_precise_servers_) server.reset();
    clear_approach_server_.reset();
    node_.reset();
    amcl_pub_.reset();
    centered_callback_group_.reset();
    peer_.reset();
  }

  void publish_amcl(
    const std::array<double, 3> & target,
    const std::string & frame_id = "map",
    bool finite = true)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = frame_id;
    message.header.stamp = peer_->now();
    message.pose.pose.position.x = target[0];
    message.pose.pose.position.y = target[1];
    message.pose.pose.orientation.z = std::sin(target[2] * 0.5);
    message.pose.pose.orientation.w = std::cos(target[2] * 0.5);
    if (!finite) {
      message.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    }
    amcl_pub_->publish(message);
  }

  amr_manipulation::SteadyTime amcl_receipt_time() const
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    return node_->amcl_pose_received_;
  }

  rclcpp::TimerBase::SharedPtr dispatch_evidence(bool publish_tf = true, bool displace_slot = false)
  {
    node_->product_.dispatch_dock = {-3.4, 0.0, std::acos(-1.0)};
    node_->product_.dispatch_approach = {-2.5, 0.0, std::acos(-1.0)};
    node_->product_.dispatch_slots[0] = {-4.1, 0.5, 0.075};
    node_->set_status(amr_interfaces::msg::ManipulatorStatus::STOWED_EMPTY, true, false, "test");
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->attachment_states_[0] = "detached";
      node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
    }
    return peer_->create_wall_timer(10ms, [this, publish_tf, displace_slot]() {
      const bool departing = retreat_server_->goal_count() != 0;
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->have_robot_pose_ = node_->have_product_pose_ = true;
      node_->robot_pose_received_ = node_->product_pose_received_ = std::chrono::steady_clock::now();
      node_->robot_pose_.pose.position.x = departing ? -2.5 : -3.6;
      node_->robot_pose_.pose.position.y = -0.07;
      node_->robot_pose_.pose.orientation.z = 1.0;
      node_->robot_pose_.pose.orientation.w = 0.0;
      node_->product_pose_.pose.position.x = -4.1 + (departing && displace_slot ? 0.04 : 0.0);
      node_->product_pose_.pose.position.y = 0.5;
      node_->product_pose_.pose.position.z = 0.075;
      if (publish_tf) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = -3.54;  // independent localization bias +0.06 m
        transform.transform.translation.y = -0.09;
        transform.transform.rotation = node_->robot_pose_.pose.orientation;
        node_->tf_buffer_.setTransform(transform, "clearance_test", false);
      }
    });
  }

  bool wait_for_amcl_receipt(
    const amr_manipulation::SteadyTime & previous_receipt,
    const std::array<double, 3> & expected_pose,
    const std::string & expected_frame)
  {
    return wait_until(
      [this, previous_receipt, expected_pose, expected_frame]() {
        std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
        if (!node_->have_amcl_pose_ || node_->amcl_pose_received_ == amr_manipulation::SteadyTime{} ||
          node_->amcl_pose_received_ <= previous_receipt ||
          node_->amcl_pose_.header.frame_id != expected_frame)
        {
          return false;
        }
        const auto & pose = node_->amcl_pose_.pose.pose;
        const double yaw = std::atan2(
          2.0 * pose.orientation.w * pose.orientation.z,
          1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
        return std::isfinite(expected_pose[0]) && std::isfinite(expected_pose[1]) &&
          std::isfinite(expected_pose[2]) && std::isfinite(pose.position.x) &&
          std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
          std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
          std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w) &&
          std::isfinite(yaw) &&
          std::abs(pose.position.x - expected_pose[0]) <= 1e-9 &&
          std::abs(pose.position.y - expected_pose[1]) <= 1e-9 &&
          std::abs(std::remainder(yaw - expected_pose[2], 2.0 * std::acos(-1.0))) <= 1e-9;
      }, 2s);
  }

  std::shared_ptr<rclcpp::Node> peer_;
  rclcpp::CallbackGroup::SharedPtr centered_callback_group_;
  std::shared_ptr<amr_manipulation::MassStageNode> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pub_;
  std::unique_ptr<NavigateActionHarness> normal_server_;
  std::unique_ptr<NavigateActionHarness> retreat_server_;
  std::unique_ptr<NavigateActionHarness> precise_server_;
  std::array<std::unique_ptr<NavigateActionHarness>,2> dispatch_servers_, dispatch_precise_servers_;
  std::unique_ptr<NavigateActionHarness> clear_approach_server_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};

class Gate6DispatchBehavior : public Gate6PickupRetreatBehavior,
  public ::testing::WithParamInterface<int> {
 protected:
  amr_manipulation::ProductSpec product_spec() override {
    auto product=test_product(); product.id=GetParam();
    product.model=GetParam()==101 ? "product_a" : "product_b";
    return product;
  }
  NavigateActionHarness & dispatch() {return *dispatch_servers_[GetParam()-101];}
  NavigateActionHarness & precise() {return *dispatch_precise_servers_[GetParam()-101];}
  void attach() {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[GetParam()-101]="attached";
    node_->attachment_state_received_[GetParam()-101]=std::chrono::steady_clock::now();
  }
};

TEST_P(Gate6DispatchBehavior, ExplicitAlignedDispatchSelectsProductBoundHeadingThenPrecision) {
  const std::array<double,3> start{-3.307,-0.023,-3.133}, target{-3.314,0.055,1.67};
  ASSERT_TRUE(node_->navigate_to(start,10s)); attach();
  ASSERT_TRUE(node_->navigate_to_aligned_precision(target,10s,true));
  ASSERT_EQ(dispatch().goal_count(),1u);
  EXPECT_EQ(dispatch().last_goal().target,(std::array<double,3>{start[0],start[1],target[2]}));
  ASSERT_EQ(precise().goal_count(),1u);
  EXPECT_EQ(precise().last_goal().target,target);
  EXPECT_EQ(dispatch().last_goal().frame_id,"map");
  EXPECT_EQ(normal_server_->goal_count(),1u);
  EXPECT_EQ(precise_server_->goal_count(),0u); EXPECT_EQ(retreat_server_->goal_count(),0u);
  EXPECT_EQ(dispatch_servers_[102-GetParam()]->goal_count(),0u);
  EXPECT_EQ(dispatch_precise_servers_[102-GetParam()]->goal_count(),0u);
}

class Gate6CenteredDockBehavior : public Gate6PickupRetreatBehavior {
 protected:
  void configure_node_before_executor_start() override {
    node_->status_timer_.reset();
  }

  amr_manipulation::ProductSpec product_spec() override {
    auto product = test_product();
    product.id = 102;
    product.model = "product_b";
    product.dispatch_dock = {-3.4, 0.0, kPi};
    product.dispatch_approach = {-2.5, 0.1, kPi};
    product.dispatch_slots[0] = {-4.1, -0.5, 0.075};
    product.dispatch_slots[1] = {-4.1, 0.0, 0.075};
    product.dispatch_slots[2] = {-4.1, 0.5, 0.075};
    product.selected_slot_index = 1;
    return product;
  }

  void SetUp() override {
    Gate6PickupRetreatBehavior::SetUp();
    set_poses_and_reseed(-2.5, 0.1, kPi, -2.56, 0.067, kPi);
    dispatch_b().set_accepted_hook([this](const auto & goal) {simulate_goal(goal);});
    clear_b().set_accepted_hook([this](const auto & goal) {simulate_clear_goal(goal);});
    precise_b().set_accepted_hook([this](const auto & goal) {simulate_goal(goal);});
    evidence_timer_ = peer_->create_wall_timer(10ms, [this]() {update_evidence();});
  }

  void TearDown() override {
    evidence_timer_.reset();
    Gate6PickupRetreatBehavior::TearDown();
  }

  NavigateActionHarness & dispatch_b() {return *dispatch_servers_[1];}
  NavigateActionHarness & clear_b() {return *clear_approach_server_;}
  NavigateActionHarness & precise_b() {return *dispatch_precise_servers_[1];}

  void set_poses(double px, double py, double pyaw, double lx, double ly, double lyaw) {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    set_pose_values_locked(px, py, pyaw, lx, ly, lyaw);
  }

  void set_poses_and_reseed(
    double px, double py, double pyaw, double lx, double ly, double lyaw)
  {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    set_pose_values_locked(px, py, pyaw, lx, ly, lyaw);
    update_evidence_locked();
  }

  void offset_physical_and_reseed(double dx, double dy, double dyaw = 0.0) {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    set_pose_values_locked(
      physical_x_.load() + dx, physical_y_.load() + dy,
      std::remainder(physical_yaw_.load() + dyaw, 2.0 * kPi),
      localized_x_.load(), localized_y_.load(), localized_yaw_.load());
    update_evidence_locked();
  }

  void enable_ros_time(int64_t nanoseconds) {
    auto * clock = node_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    jump_ros_time(nanoseconds);
  }

  void jump_ros_time(int64_t nanoseconds) {
    EXPECT_EQ(rcl_set_ros_time_override(
        node_->get_clock()->get_clock_handle(), nanoseconds), RCL_RET_OK);
    update_evidence();
  }

  std::future<bool> start_dock(const std::array<double, 3> & stance) {
    return std::async(std::launch::async, [this, stance]() {
      amr_manipulation::Product102CenteredDockEvidence dock;
      return node_->navigate_product102_centered_dock(stance, 120s, dock);
    });
  }

  std::array<double, 3> centered_stance() {
    return amr_interfaces::placement::final_placement_stance(
      102, node_->product_.dispatch_dock,
      node_->product_.dispatch_slots[node_->product_.selected_slot_index]).physical;
  }

  void inject_physical_entry_defect(int defect) {
    refresh_physical_ = false;
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex_);
    std::lock_guard<std::mutex> evidence_lock(node_->evidence_mutex_);
    auto & physical = node_->robot_pose_;
    const auto wall_now = std::chrono::steady_clock::now();
    node_->have_robot_pose_ = true;
    node_->robot_pose_received_ = wall_now;
    physical.header.frame_id = "factory_world";
    physical.pose.position.x = physical_x_.load();
    physical.pose.position.y = physical_y_.load();
    physical.pose.position.z = 0.0;
    physical.pose.orientation.x = 0.0;
    physical.pose.orientation.y = 0.0;
    physical.pose.orientation.z = std::sin(physical_yaw_.load() * 0.5);
    physical.pose.orientation.w = std::cos(physical_yaw_.load() * 0.5);
    if (defect == 1) {
      physical.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    } else if (defect == 2) {
      physical.pose.orientation.z = 0.0;
      physical.pose.orientation.w = 0.0;
    } else if (defect == 3) {
      node_->robot_pose_received_ = wall_now - 201ms;
    } else if (defect == 4) {
      node_->robot_pose_received_ = wall_now + 1s;
    }
  }

  bool fresh_ready_base_odom_attachment() {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    const auto wall_now = std::chrono::steady_clock::now();
    const auto fresh = [wall_now](const auto & receipt) {
        return receipt != amr_manipulation::SteadyTime{} && wall_now >= receipt &&
          wall_now - receipt <= 200ms;
      };
    return node_->have_base_ && node_->have_odometry_ &&
      fresh(node_->base_received_) && fresh(node_->odometry_received_) &&
      node_->base_status_.valid && node_->base_status_.source_boot_id != 0U &&
      node_->base_status_.sequence != 0U &&
      node_->base_status_.state == amr_interfaces::msg::BaseStatus::READY &&
      node_->base_status_.reason == amr_interfaces::msg::BaseStatus::REASON_READY &&
      node_->attachment_states_[1] == "attached" &&
      fresh(node_->attachment_state_received_[1]);
  }

  bool fresh_valid_physical_pose() {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    const auto wall_now = std::chrono::steady_clock::now();
    double yaw = 0.0;
    return node_->have_robot_pose_ && node_->robot_pose_received_ !=
      amr_manipulation::SteadyTime{} && wall_now >= node_->robot_pose_received_ &&
      wall_now - node_->robot_pose_received_ <= 200ms &&
      node_->robot_pose_.header.frame_id == "factory_world" &&
      node_->unit_pose_yaw(node_->robot_pose_.pose, yaw);
  }

  bool fresh_current_tf() {
    amr_manipulation::CurrentTfEvidence localized;
    return node_->latest_current_tf_pose(localized);
  }

  void set_tf_defect_and_reseed(int defect)
  {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    node_->tf_buffer_.enable_scaled_return(defect == 5);
    tf_defect_.store(defect);
    node_->tf_buffer_.clear();
    update_evidence_locked();
  }

  void set_pose_values_locked(
    double px, double py, double pyaw, double lx, double ly, double lyaw)
  {
    physical_x_.store(px);
    physical_y_.store(py);
    physical_yaw_.store(pyaw);
    localized_x_.store(lx);
    localized_y_.store(ly);
    localized_yaw_.store(lyaw);
  }

  void simulate_goal(const nav2_msgs::action::NavigateToPose::Goal & goal) {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    simulate_goal_locked(goal);
    update_evidence_locked();
  }

  void simulate_goal_locked(const nav2_msgs::action::NavigateToPose::Goal & goal) {
    const double bias_x = physical_x_.load() - localized_x_.load();
    const double bias_y = physical_y_.load() - localized_y_.load();
    const double bias_yaw = std::remainder(
      physical_yaw_.load() - localized_yaw_.load(), 2.0 * kPi);
    const double goal_yaw = std::atan2(
      2.0 * goal.pose.pose.orientation.w * goal.pose.pose.orientation.z,
      1.0 - 2.0 * goal.pose.pose.orientation.z * goal.pose.pose.orientation.z);
    set_pose_values_locked(goal.pose.pose.position.x + bias_x, goal.pose.pose.position.y + bias_y,
      std::remainder(goal_yaw + bias_yaw, 2.0 * kPi),
      goal.pose.pose.position.x, goal.pose.pose.position.y, goal_yaw);
  }

  void simulate_clear_goal(const nav2_msgs::action::NavigateToPose::Goal & goal) {
    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    simulate_goal_locked(goal);
    const auto attempt = clear_accept_count_.fetch_add(1);
    const int mode = clear_miss_mode_.load();
    if (mode == 2 || (mode == 1 && attempt == 0)) {
      set_pose_values_locked(physical_x_.load() + 0.012, physical_y_.load(), physical_yaw_.load(),
        localized_x_.load(), localized_y_.load(), localized_yaw_.load());
    }
    update_evidence_locked();
  }

  void update_evidence() {
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex_);
    update_evidence_locked();
  }

  void update_evidence_locked() {
    const auto wall_now = std::chrono::steady_clock::now();
    const double px = physical_x_.load();
    const double py = physical_y_.load();
    const double pyaw = physical_yaw_.load();
    const double lx = localized_x_.load();
    const double ly = localized_y_.load();
    const double lyaw = localized_yaw_.load();
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      if (refresh_physical_.load()) {
        node_->have_robot_pose_ = true;
        node_->robot_pose_received_ = wall_now;
        node_->robot_pose_.header.frame_id = physical_frame_valid_.load() ?
          "factory_world" : "odom";
        node_->robot_pose_.header.stamp = node_->now();
        node_->robot_pose_.pose.position.x = px;
        node_->robot_pose_.pose.position.y = py;
        node_->robot_pose_.pose.position.z = 0.0;
        node_->robot_pose_.pose.orientation.x = 0.0;
        node_->robot_pose_.pose.orientation.y = 0.0;
        node_->robot_pose_.pose.orientation.z = std::sin(pyaw * 0.5);
        node_->robot_pose_.pose.orientation.w = std::cos(pyaw * 0.5);
      }
      node_->have_base_ = node_->have_odometry_ = true;
      node_->base_received_ = wall_now - std::chrono::milliseconds(base_receipt_age_ms_.load());
      node_->odometry_received_ =
        wall_now - std::chrono::milliseconds(odometry_receipt_age_ms_.load());
      node_->base_status_.valid = base_status_valid_.load();
      node_->base_status_.source_boot_id = base_boot_valid_.load() ? 81U : 0U;
      node_->base_status_.sequence = base_sequence_valid_.load() ? ++base_sequence_ : 0U;
      node_->base_status_.state = base_ready_.load() ?
        amr_interfaces::msg::BaseStatus::READY :
        static_cast<decltype(node_->base_status_.state)>(0);
      node_->base_status_.reason = base_reason_ready_.load() ?
        amr_interfaces::msg::BaseStatus::REASON_READY :
        static_cast<decltype(node_->base_status_.reason)>(0);
      node_->odometry_.twist.twist.linear.x = odom_x_.load();
      node_->odometry_.twist.twist.linear.y = odom_y_.load();
      node_->odometry_.twist.twist.angular.z = odom_yaw_.load();
      node_->attachment_states_[1] = attachment_lost_.load() ? "detached" : "attached";
      node_->attachment_state_received_[1] = wall_now;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = lx;
    transform.transform.translation.y = ly;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.z = std::sin(lyaw * 0.5);
    transform.transform.rotation.w = std::cos(lyaw * 0.5);
    const int tf_defect = tf_defect_.load();
    if (tf_defect == 1) {
      node_->tf_buffer_.clear();
      return;
    }
    if (tf_defect == 2) transform.header.stamp = node_->now() - rclcpp::Duration(0, 400000000);
    if (tf_defect == 3) transform.header.stamp = node_->now() + rclcpp::Duration(0, 100000000);
    if (tf_defect == 4) transform.header.stamp = builtin_interfaces::msg::Time{};
    if (tf_defect == 6) transform.transform.translation.x =
      std::numeric_limits<double>::quiet_NaN();
    if (tf_defect == 7) transform.transform.rotation.z =
      std::numeric_limits<double>::quiet_NaN();
    (void)node_->tf_buffer_.setTransform(transform, "centered_dock_behavior", false);
  }

  static constexpr double kPi = 3.14159265358979323846;
  std::mutex pose_update_mutex_;
  std::atomic<double> physical_x_{-2.5}, physical_y_{0.1}, physical_yaw_{kPi};
  std::atomic<double> localized_x_{-2.56}, localized_y_{0.067}, localized_yaw_{kPi};
  std::atomic_int clear_miss_mode_{0}, tf_defect_{0};
  std::atomic_uint clear_accept_count_{0};
  std::atomic<double> odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};
  std::atomic_bool refresh_physical_{true}, physical_frame_valid_{true}, attachment_lost_{false};
  std::atomic_bool base_status_valid_{true}, base_boot_valid_{true}, base_sequence_valid_{true};
  std::atomic_bool base_ready_{true}, base_reason_ready_{true};
  std::atomic_int base_receipt_age_ms_{0}, odometry_receipt_age_ms_{0};
  std::atomic_uint32_t base_sequence_{0};
  rclcpp::TimerBase::SharedPtr evidence_timer_;
};

TEST_F(Gate6CenteredDockBehavior, PrecisionTargetUsesPostArrivalCurrentTfBias) {
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  dispatch_b().set_accepted_hook([this](const auto &) {
    // Physical motion is +5/+2 mm while current TF moves +37/+26 mm. The
    // stopped post-heading bias therefore changes by -32/-24 mm.
    set_poses_and_reseed(-2.495, 0.102, kPi, -2.523, 0.093, kPi);
  });
  amr_manipulation::Product102CenteredDockEvidence dock;
  decltype(node_->state_) state_before{};
  decltype(node_->base_allowed_) base_allowed_before{};
  decltype(node_->attached_) attached_before{};
  std::string detail_before;
  decltype(node_->sequence_) sequence_before{};
  {
    std::lock_guard<std::mutex> lock(node_->status_mutex_);
    state_before = node_->state_;
    base_allowed_before = node_->base_allowed_;
    attached_before = node_->attached_;
    detail_before = node_->detail_;
    sequence_before = node_->sequence_;
  }
  const bool dock_succeeded =
    node_->navigate_product102_centered_dock(stance.physical, 10s, dock);
  {
    std::lock_guard<std::mutex> lock(node_->status_mutex_);
    EXPECT_EQ(node_->state_, state_before);
    EXPECT_EQ(node_->base_allowed_, base_allowed_before);
    EXPECT_EQ(node_->attached_, attached_before);
    EXPECT_EQ(node_->detail_, detail_before);
    EXPECT_EQ(node_->sequence_, sequence_before);
  }
  ASSERT_TRUE(dock_succeeded);
  ASSERT_EQ(dispatch_b().goal_count(), 1U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
  ASSERT_EQ(precise_b().goal_count(), 1U);
  const auto actual = precise_b().last_goal().target;
  EXPECT_NEAR(actual[0], -3.380, 1e-9);
  EXPECT_NEAR(actual[1], 0.091, 1e-9);
  EXPECT_NEAR(actual[2], stance.physical[2], 1e-9);
  EXPECT_NEAR(dock.physical.pose.position.x, stance.physical[0], 1e-9);
  EXPECT_NEAR(dock.physical.pose.position.y, stance.physical[1], 1e-9);
}

TEST_F(Gate6CenteredDockBehavior, ClearPathUsesNearestForwardAndReverseTangents) {
  std::mutex yaw_mutex;
  std::vector<double> dispatch_yaws;
  dispatch_b().set_accepted_hook([this, &yaw_mutex, &dispatch_yaws](const auto & goal) {
    const double yaw = std::atan2(
      2.0 * goal.pose.pose.orientation.w * goal.pose.pose.orientation.z,
      1.0 - 2.0 * goal.pose.pose.orientation.z * goal.pose.pose.orientation.z);
    {
      std::lock_guard<std::mutex> lock(yaw_mutex);
      dispatch_yaws.push_back(yaw);
    }
    simulate_goal(goal);
  });
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  amr_manipulation::Product102CenteredDockEvidence dock;
  ASSERT_TRUE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  ASSERT_EQ(clear_b().goal_count(), 1U);
  ASSERT_EQ(precise_b().goal_count(), 1U);
  {
    std::lock_guard<std::mutex> lock(yaw_mutex);
    ASSERT_EQ(dispatch_yaws.size(), 2U);
    EXPECT_NEAR(dispatch_yaws[0], kPi * 0.5, 1e-9);
  }

  set_poses_and_reseed(-2.5, 0.0, -kPi * 0.5, -2.56, -0.033, -kPi * 0.5);
  ASSERT_TRUE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  ASSERT_EQ(clear_b().goal_count(), 2U);
  ASSERT_EQ(precise_b().goal_count(), 2U);
  {
    std::lock_guard<std::mutex> lock(yaw_mutex);
    ASSERT_EQ(dispatch_yaws.size(), 4U);
    EXPECT_NEAR(dispatch_yaws[2], -kPi * 0.5, 1e-9);
  }
}

TEST_F(Gate6CenteredDockBehavior, BoundaryPostTangentBiasChangesClearTarget) {
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  std::atomic_uint dispatch_accept_count{0};
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  dispatch_b().set_accepted_hook([this, &dispatch_accept_count](const auto & goal) {
    if (dispatch_accept_count.fetch_add(1) == 0U) {
      // Keep physical pose fixed at tangent acceptance while current TF moves.
      set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.528, -0.009, kPi * 0.5);
    } else {
      simulate_goal(goal);
    }
  });

  amr_manipulation::Product102CenteredDockEvidence dock;
  ASSERT_TRUE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  ASSERT_EQ(dispatch_b().goal_count(), 2U);
  ASSERT_EQ(clear_b().goal_count(), 1U);
  ASSERT_EQ(precise_b().goal_count(), 1U);
  const auto clear_target = clear_b().last_goal().target;
  EXPECT_NEAR(clear_target[0], -2.528, 1e-9);
  EXPECT_NEAR(clear_target[1], 0.091, 1e-9);
  const auto dock_target = precise_b().last_goal().target;
  EXPECT_NEAR(dock_target[0], stance.physical[0] - 0.028, 1e-9);
  EXPECT_NEAR(dock_target[1], stance.physical[1] - 0.009, 1e-9);
  EXPECT_NEAR(dock_target[2], stance.physical[2], 1e-9);
  EXPECT_NEAR(dock.physical.pose.position.x, stance.physical[0], 1e-9);
  EXPECT_NEAR(dock.physical.pose.position.y, stance.physical[1], 1e-9);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
  EXPECT_EQ(dispatch_servers_[0]->goal_count(), 0U);
  EXPECT_EQ(dispatch_precise_servers_[0]->goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, BoundaryPhysicalDockResiduals) {
  // Only this numeric-boundary test uses yaw 0 and a zero-Y immutable stance:
  // the unchanged centered stance helper then represents physical 0.010 m
  // exactly. The registered-scene fixture defaults remain untouched elsewhere.
  node_->product_.dispatch_dock = {-3.4, 0.1, 0.0};
  node_->product_.dispatch_approach = {-2.5, 0.1, 0.0};
  node_->product_.dispatch_slots[1] = {-4.1, 0.1, 0.075};
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  ASSERT_TRUE(stance.product102_center_slot);
  ASSERT_DOUBLE_EQ(stance.lateral_offset, 0.0);
  ASSERT_DOUBLE_EQ(stance.physical[1], 0.0);
  ASSERT_DOUBLE_EQ(stance.physical[2], 0.0);

  const std::array<double, 4> requested_residuals{0.0, 0.009, 0.010, 0.0100001};
  for (const double requested_residual : requested_residuals) {
    set_poses_and_reseed(-2.5, 0.0, 0.0, -2.5, 0.0, 0.0);
    precise_b().set_accepted_hook([this, requested_residual](const auto & goal) {
      simulate_goal(goal);
      offset_physical_and_reseed(0.0, requested_residual);
    });
    const auto dispatch_before = dispatch_b().goal_count();
    const auto clear_before = clear_b().goal_count();
    const auto precise_before = precise_b().goal_count();
    amr_manipulation::Product102CenteredDockEvidence dock;
    const bool helper_result =
      node_->navigate_product102_centered_dock(stance.physical, 10s, dock);
    EXPECT_EQ(helper_result, requested_residual <= 0.010);

    ASSERT_TRUE(fresh_valid_physical_pose());
    double actual_residual = 0.0;
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      actual_residual = std::hypot(
        node_->robot_pose_.pose.position.x - stance.physical[0],
        node_->robot_pose_.pose.position.y - stance.physical[1]);
    }
    EXPECT_EQ(actual_residual, requested_residual);
    const auto admission = amr_manipulation::centered_b_alignment_admission(actual_residual);
    EXPECT_EQ(admission, requested_residual <= 0.010 ?
      amr_manipulation::CenteredBAlignmentAdmission::DOCK_ONLY :
      amr_manipulation::CenteredBAlignmentAdmission::REJECT);
    EXPECT_EQ(dispatch_b().goal_count(), dispatch_before + 1U);
    EXPECT_EQ(clear_b().goal_count(), clear_before);
    EXPECT_EQ(precise_b().goal_count(), precise_before + 1U);
    const auto dock_target = precise_b().last_goal().target;
    EXPECT_DOUBLE_EQ(dock_target[0], stance.physical[0]);
    EXPECT_DOUBLE_EQ(dock_target[1], stance.physical[1]);
    EXPECT_GT(std::hypot(dock_target[0] - (-2.5), dock_target[1]), 1.0);
  }
  precise_b().set_accepted_hook([this](const auto & goal) {simulate_goal(goal);});
}

TEST_F(Gate6CenteredDockBehavior, BoundaryCurrentTfDefectsRejectBeforeNavigation) {
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  for (int defect = 2; defect <= 7; ++defect) {
    physical_frame_valid_ = true;
    refresh_physical_ = true;
    attachment_lost_ = false;
    set_poses_and_reseed(-2.5, 0.1, kPi, -2.56, 0.067, kPi);
    set_tf_defect_and_reseed(0);
    ASSERT_TRUE(fresh_valid_physical_pose());
    ASSERT_TRUE(fresh_ready_base_odom_attachment());
    ASSERT_TRUE(fresh_current_tf());

    set_tf_defect_and_reseed(defect);
    ASSERT_EQ(tf_defect_.load(), defect);
    ASSERT_TRUE(fresh_valid_physical_pose());
    ASSERT_TRUE(fresh_ready_base_odom_attachment());
    EXPECT_FALSE(fresh_current_tf());
    if (defect == 5) {
      const auto native = node_->tf_buffer_.native_lookup_transform(
        "map", "base_footprint", tf2::TimePointZero);
      const auto & native_q = native.transform.rotation;
      ASSERT_NEAR(std::hypot(std::hypot(native_q.x, native_q.y),
        std::hypot(native_q.z, native_q.w)), 1.0, 1e-9);
      const auto intercepted = node_->tf_buffer_.lookupTransform(
        "map", "base_footprint", tf2::TimePointZero);
      const auto & intercepted_q = intercepted.transform.rotation;
      ASSERT_NEAR(std::hypot(std::hypot(intercepted_q.x, intercepted_q.y),
        std::hypot(intercepted_q.z, intercepted_q.w)), 1.0001, 1e-9);
    } else if (defect == 2 || defect == 3 || defect == 4) {
      const auto injected = node_->tf_buffer_.lookupTransform(
        "map", "base_footprint", tf2::TimePointZero);
      const int64_t injected_stamp_ns =
        static_cast<int64_t>(injected.header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(injected.header.stamp.nanosec);
      const int64_t now_ns = node_->now().nanoseconds();
      if (defect == 2) {
        EXPECT_LT(injected_stamp_ns, now_ns);
        EXPECT_GT(static_cast<double>(now_ns - injected_stamp_ns) / 1e9, 0.30);
      } else if (defect == 3) {
        EXPECT_GT(injected_stamp_ns, now_ns);
      } else {
        EXPECT_EQ(injected.header.stamp.sec, 0);
        EXPECT_EQ(injected.header.stamp.nanosec, 0U);
      }
    } else {
      // TF2 may reject malformed values during insertion; the fixture mode
      // above records the intended malformed input and the current-TF gate
      // must still observe no admissible transform.
      try {
        const auto injected = node_->tf_buffer_.lookupTransform(
          "map", "base_footprint", tf2::TimePointZero);
        if (defect == 6) {
          EXPECT_FALSE(std::isfinite(injected.transform.translation.x));
        } else {
          EXPECT_FALSE(std::isfinite(injected.transform.rotation.z));
        }
      } catch (const tf2::TransformException &) {
        SUCCEED();
      }
    }
    amr_manipulation::Product102CenteredDockEvidence dock;
    EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
    EXPECT_EQ(dispatch_b().goal_count(), 0U);
    EXPECT_EQ(clear_b().goal_count(), 0U);
    EXPECT_EQ(precise_b().goal_count(), 0U);
  }
  set_tf_defect_and_reseed(0);
  ASSERT_TRUE(fresh_current_tf());
}

TEST_F(Gate6CenteredDockBehavior, BoundaryInvalidPhysicalEvidenceRejectsBeforeNavigation) {
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  for (int defect = 1; defect <= 4; ++defect) {
    refresh_physical_ = true;
    physical_frame_valid_ = true;
    attachment_lost_ = false;
    tf_defect_ = 0;
    set_poses_and_reseed(-2.5, 0.1, kPi, -2.56, 0.067, kPi);
    ASSERT_TRUE(fresh_valid_physical_pose());
    ASSERT_TRUE(fresh_ready_base_odom_attachment());
    ASSERT_TRUE(fresh_current_tf());

    inject_physical_entry_defect(defect);
    update_evidence();
    EXPECT_FALSE(refresh_physical_.load());
    EXPECT_TRUE(fresh_ready_base_odom_attachment());
    EXPECT_TRUE(fresh_current_tf());
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      const auto wall_now = std::chrono::steady_clock::now();
      const auto & physical = node_->robot_pose_;
      ASSERT_TRUE(node_->have_robot_pose_);
      ASSERT_EQ(physical.header.frame_id, "factory_world");
      if (defect == 1) {
        EXPECT_FALSE(std::isfinite(physical.pose.position.x));
      } else if (defect == 2) {
        EXPECT_DOUBLE_EQ(physical.pose.orientation.z, 0.0);
        EXPECT_DOUBLE_EQ(physical.pose.orientation.w, 0.0);
      } else if (defect == 3) {
        ASSERT_LE(node_->robot_pose_received_, wall_now);
        EXPECT_GT(wall_now - node_->robot_pose_received_, 200ms);
      } else {
        EXPECT_LT(wall_now, node_->robot_pose_received_);
      }
    }
    amr_manipulation::Product102CenteredDockEvidence dock;
    EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
    EXPECT_EQ(dispatch_b().goal_count(), 0U);
    EXPECT_EQ(clear_b().goal_count(), 0U);
    EXPECT_EQ(precise_b().goal_count(), 0U);
  }
  refresh_physical_ = true;
  update_evidence();
}

TEST_F(Gate6CenteredDockBehavior, BoundaryCommandedClearTranslationBound) {
  set_poses_and_reseed(-2.5, -0.051, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  EXPECT_NEAR(std::hypot(-2.5 - (-2.5), 0.1 - (-0.051)), 0.151, 1e-12);
  EXPECT_LE(std::hypot(-2.5 - (-2.5), 0.1 - (-0.051)), 0.155);
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 1U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
  EXPECT_NEAR(dispatch_b().last_goal().target[2], kPi * 0.5, 1e-9);
  EXPECT_NEAR(std::hypot(-2.5 - physical_x_.load(), 0.1 - physical_y_.load()), 0.151, 1e-12);
  EXPECT_TRUE(fresh_valid_physical_pose());
  EXPECT_TRUE(fresh_current_tf());
}

TEST_F(Gate6CenteredDockBehavior, BoundaryOriginalReferenceDriftCancelsExactGoal) {
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  std::atomic_uint dispatch_accept_count{0};
  dispatch_b().set_accepted_hook([this, &dispatch_accept_count](const auto & goal) {
    if (dispatch_accept_count.fetch_add(1) == 0U) {
      simulate_goal(goal);
      dispatch_b().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
    }
  });
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  auto result = std::async(std::launch::async, [this, stance]() {
    amr_manipulation::Product102CenteredDockEvidence dock;
    return node_->navigate_product102_centered_dock(stance.physical, 10s, dock);
  });

  bool arrival_accepted = wait_until([this]() {
      if (dispatch_b().goal_count() < 2U) return false;
      const auto arrival = dispatch_b().last_goal();
      return dispatch_b().was_accepted(arrival.uuid);
    }, 3000ms);
  if (arrival_accepted) {
    offset_physical_and_reseed(0.113, 0.0);
  } else {
    node_->cancel_requested_.store(true);
  }
  auto completed = result.wait_for(5s);
  if (completed != std::future_status::ready) {
    node_->cancel_requested_.store(true);
    completed = result.wait_for(5s);
  }
  ASSERT_EQ(completed, std::future_status::ready);
  EXPECT_FALSE(result.get());
  ASSERT_TRUE(arrival_accepted);

  const auto arrival = dispatch_b().last_goal();
  EXPECT_GT(std::hypot(0.113, 0.1), 0.15);
  EXPECT_LT(0.113, 0.15);
  EXPECT_TRUE(fresh_valid_physical_pose());
  EXPECT_TRUE(dispatch_b().was_accepted(arrival.uuid));
  EXPECT_TRUE(dispatch_b().was_canceled(arrival.uuid));
  EXPECT_TRUE(wait_until([this, &arrival]() {
      return dispatch_b().was_terminal_canceled(arrival.uuid);
    }, 2s));
  EXPECT_TRUE(dispatch_b().was_terminal_canceled(arrival.uuid));
  EXPECT_EQ(clear_b().goal_count(), 1U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, BoundaryArrivalHeadingMissCannotRepair) {
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  std::atomic_uint dispatch_accept_count{0};
  dispatch_b().set_accepted_hook([this, &dispatch_accept_count](const auto & goal) {
    if (dispatch_accept_count.fetch_add(1) == 1U) {
      simulate_goal(goal);
      offset_physical_and_reseed(0.0, 0.0, 0.151);
    } else {
      simulate_goal(goal);
    }
  });
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 2U);
  EXPECT_EQ(clear_b().goal_count(), 1U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
  EXPECT_TRUE(fresh_valid_physical_pose());
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    double physical_yaw = 0.0;
    ASSERT_TRUE(node_->unit_pose_yaw(node_->robot_pose_.pose, physical_yaw));
    EXPECT_NEAR(std::hypot(
        node_->robot_pose_.pose.orientation.x,
        node_->robot_pose_.pose.orientation.y), 0.0, 1e-12);
    EXPECT_NEAR(std::hypot(
        node_->robot_pose_.pose.orientation.z,
        node_->robot_pose_.pose.orientation.w), 1.0, 1e-12);
    EXPECT_NEAR(std::abs(std::remainder(physical_yaw - stance.physical[2], 2.0 * kPi)),
      0.151, 1e-9);
  }
}

#ifdef AMR_GATE6_CENTERED_COVERAGE_TESTS
struct ScopedCleanup {
  explicit ScopedCleanup(std::function<void()> action) : action_(std::move(action)) {}
  ~ScopedCleanup() {action_();}
  std::function<void()> action_;
};

// Shared ROS-time budget (120 s): parameter false = forward expiry, true = rollback.
class Gate6CenteredBudgetBehavior : public Gate6CenteredDockBehavior,
  public ::testing::WithParamInterface<bool> {
 protected:
  static constexpr int64_t kBaseNs = 100000000000LL;
  int64_t violating_ns() const {return GetParam() ? 90000000000LL : 221000000000LL;}
};

TEST_P(Gate6CenteredBudgetBehavior, InitialStopObservationHonorsSharedBudget) {
  enable_ros_time(kBaseNs);
  odom_x_ = 0.05;  // never stationary, so the call stays in the initial stop wait
  update_evidence();
  const auto began = std::chrono::steady_clock::now();
  auto result = start_dock(centered_stance());
  ScopedCleanup cleanup([this]() {node_->cancel_requested_.store(true);});
  ASSERT_EQ(result.wait_for(300ms), std::future_status::timeout);
  jump_ros_time(violating_ns());
  ASSERT_EQ(result.wait_for(3s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_LT(std::chrono::steady_clock::now() - began, 4s);  // observer wall limit is 8 s
  EXPECT_EQ(dispatch_b().goal_count(), 0U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_P(Gate6CenteredBudgetBehavior, IntermediateTangentStageHonorsSharedBudget) {
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  enable_ros_time(kBaseNs);
  dispatch_b().set_accepted_hook([this](const auto & goal) {
    simulate_goal(goal);
    jump_ros_time(violating_ns());
  });
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(centered_stance(), 120s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 1U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_P(Gate6CenteredBudgetBehavior, IntermediateClearStageHonorsSharedBudget) {
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  enable_ros_time(kBaseNs);
  clear_b().set_accepted_hook([this](const auto & goal) {
    simulate_clear_goal(goal);
    jump_ros_time(violating_ns());
  });
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(centered_stance(), 120s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 1U);  // no arrival heading after the clear stage
  EXPECT_EQ(clear_b().goal_count(), 1U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_P(Gate6CenteredBudgetBehavior, ActiveLongDockCancelsExactGoalOnBudgetViolation) {
  enable_ros_time(kBaseNs);
  precise_b().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  precise_b().set_accepted_hook([this](const auto & goal) {
    simulate_goal(goal);
    jump_ros_time(violating_ns());
  });
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(centered_stance(), 120s, dock));
  ASSERT_EQ(precise_b().goal_count(), 1U);
  const auto dock_goal = precise_b().last_goal();
  EXPECT_TRUE(precise_b().was_accepted(dock_goal.uuid));
  EXPECT_TRUE(precise_b().was_canceled(dock_goal.uuid));
  EXPECT_TRUE(wait_until([this, &dock_goal]() {
      return precise_b().was_terminal_canceled(dock_goal.uuid);
    }, 2s));
  EXPECT_EQ(dispatch_b().goal_count(), 1U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
}

TEST_P(Gate6CenteredBudgetBehavior, PostDockObservationHonorsSharedBudgetAfterCompletedGoal) {
  enable_ros_time(kBaseNs);
  std::atomic_size_t dispatch_goals_at_dock{0};
  std::atomic_size_t clear_goals_at_dock{0};
  precise_b().set_accepted_hook([this, &dispatch_goals_at_dock, &clear_goals_at_dock](
      const auto & goal) {
    simulate_goal(goal);
    dispatch_goals_at_dock.store(dispatch_b().goal_count());
    clear_goals_at_dock.store(clear_b().goal_count());
  });
  std::atomic_bool hook_fired{false};
  std::atomic_uint hook_fire_count{0};
  node_->tf_buffer_.set_post_lookup_hook([this, &hook_fired, &hook_fire_count]() {
    if (precise_b().goal_count() != 1U) return;
    const auto dock_goal = precise_b().last_goal();
    if (!precise_b().was_terminal_succeeded(dock_goal.uuid)) return;
    bool expected = false;
    if (!hook_fired.compare_exchange_strong(expected, true)) return;
    hook_fire_count.fetch_add(1);
    jump_ros_time(violating_ns());
    node_->tf_buffer_.set_post_lookup_hook({});
  });

  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(centered_stance(), 120s, dock));
  ASSERT_EQ(precise_b().goal_count(), 1U);
  const auto dock_goal = precise_b().last_goal();
  EXPECT_TRUE(precise_b().was_terminal_succeeded(dock_goal.uuid));
  EXPECT_EQ(precise_b().canceled_count(), 0U);  // completed result, not active cancellation
  EXPECT_TRUE(hook_fired.load());
  EXPECT_EQ(hook_fire_count.load(), 1U);
  ASSERT_EQ(dispatch_goals_at_dock.load(), 1U);
  ASSERT_EQ(clear_goals_at_dock.load(), 0U);
  EXPECT_EQ(dispatch_b().goal_count(), dispatch_goals_at_dock.load());
  EXPECT_EQ(clear_b().goal_count(), clear_goals_at_dock.load());
}

TEST_P(Gate6CenteredBudgetBehavior, CumulativeBudgetCancelsExactLongDockGoal) {
  if (GetParam())
    GTEST_SKIP() << "cumulative-stage sequence exercises forward expiry only";

  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  enable_ros_time(kBaseNs);
  std::atomic_uint dispatch_accept_count{0};
  dispatch_b().set_accepted_hook([this, &dispatch_accept_count](const auto & goal) {
    simulate_goal(goal);
    if (dispatch_accept_count.fetch_add(1) == 0)
      jump_ros_time(150000000000LL);
  });
  clear_b().set_accepted_hook([this](const auto & goal) {
    simulate_clear_goal(goal);
    jump_ros_time(200000000000LL);
  });
  precise_b().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  precise_b().set_accepted_hook([this](const auto & goal) {
    simulate_goal(goal);
    jump_ros_time(225000000000LL);
  });

  auto result = start_dock(centered_stance());
  ScopedCleanup cleanup([this]() {node_->cancel_requested_.store(true);});
  ASSERT_EQ(result.wait_for(4s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  ASSERT_EQ(precise_b().goal_count(), 1U);
  const auto dock_goal = precise_b().last_goal();
  EXPECT_TRUE(precise_b().was_accepted(dock_goal.uuid));
  EXPECT_TRUE(precise_b().was_canceled(dock_goal.uuid));
  ASSERT_TRUE(wait_until([this, &dock_goal]() {
      return precise_b().was_terminal_canceled(dock_goal.uuid);
    }, 2s));

  EXPECT_EQ(dispatch_b().goal_count(), 2U);  // tangent and arrival only
  EXPECT_EQ(clear_b().goal_count(), 1U);
  EXPECT_EQ(precise_b().goal_count(), 1U);  // no action follows the held dock
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
  EXPECT_EQ(precise_server_->goal_count(), 0U);
  EXPECT_EQ(dispatch_servers_[0]->goal_count(), 0U);
  EXPECT_EQ(dispatch_precise_servers_[0]->goal_count(), 0U);
}

INSTANTIATE_TEST_SUITE_P(ExpiryAndRollback, Gate6CenteredBudgetBehavior, ::testing::Bool());

// Withheld goal acceptance: param = (cause, server). cause 0 expiry, 1 rollback,
// 2 cooperative cancel; server 0 arrival-heading dispatch, 1 long-dock precise.
class Gate6CenteredDelayedAcceptanceBehavior : public Gate6CenteredDockBehavior,
  public ::testing::WithParamInterface<std::tuple<int, int>> {
 protected:
  bool use_reentrant_centered_callback_group() const override {return true;}
};

TEST_P(Gate6CenteredDelayedAcceptanceBehavior, WithheldAcceptanceFailsAndCancelsExactLateGoal) {
  const int cause = std::get<0>(GetParam());
  NavigateActionHarness & server = std::get<1>(GetParam()) == 0 ? dispatch_b() : precise_b();
  server.set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  const auto barrier = std::make_shared<NavigateActionHarness::RequestBarrier>();
  server.set_request_barrier(barrier);
  if (cause < 2) enable_ros_time(100000000000LL);
  auto result = start_dock(centered_stance());
  ScopedCleanup cleanup([this, barrier]() {
    barrier->release();
    node_->cancel_requested_.store(true);
  });
  ASSERT_TRUE(barrier->wait_for_request(5s));
  const auto uuid = barrier->requested_uuid();
  EXPECT_EQ(server.goal_count(), 0U);  // acceptance is withheld

  // Heartbeat must keep running while the request callback is blocked.
  amr_manipulation::SteadyTime physical_before;
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    physical_before = node_->robot_pose_received_;
  }
  ASSERT_TRUE(wait_until([this, physical_before]() {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      return node_->robot_pose_received_ > physical_before;
    }, 500ms));

  if (cause == 0) jump_ros_time(221000000000LL);
  if (cause == 1) jump_ros_time(90000000000LL);
  if (cause == 2) node_->cancel_requested_.store(true);
  // The helper must fail while acceptance is withheld for every cause.
  ASSERT_EQ(result.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_EQ(server.goal_count(), 0U);
  barrier->release();
  EXPECT_TRUE(wait_until([&]() {return server.was_canceled(uuid);}, 3s));
  EXPECT_TRUE(wait_until([&]() {return server.was_terminal_canceled(uuid);}, 3s));
  EXPECT_TRUE(server.was_accepted(uuid));
  EXPECT_EQ(server.goal_count(), 1U);  // only the late goal; no later action
  EXPECT_EQ(clear_b().goal_count(), 0U);
  if (std::get<1>(GetParam()) == 0) {
    EXPECT_EQ(precise_b().goal_count(), 0U);
  } else {
    EXPECT_EQ(dispatch_b().goal_count(), 1U);
  }
}

INSTANTIATE_TEST_SUITE_P(
  CausesAndServers, Gate6CenteredDelayedAcceptanceBehavior,
  ::testing::Combine(::testing::Range(0, 3), ::testing::Range(0, 2)));

TEST_F(Gate6CenteredDockBehavior, ObserverRejectsNonStationaryAxesAndInvalidEvidence) {
  const auto stance = centered_stance();
  const std::vector<std::pair<std::string, std::function<void()>>> defects{
    {"linear_x", [this]() {odom_x_ = 0.0101;}},
    {"linear_y", [this]() {odom_y_ = -0.0101;}},
    {"angular_z", [this]() {odom_yaw_ = 0.0101;}},
    {"nonfinite", [this]() {odom_x_ = std::numeric_limits<double>::quiet_NaN();}},
    {"base_invalid", [this]() {base_status_valid_ = false;}},
    {"boot_zero", [this]() {base_boot_valid_ = false;}},
    {"sequence_zero", [this]() {base_sequence_valid_ = false;}},
    {"state_not_ready", [this]() {base_ready_ = false;}},
    {"reason_not_ready", [this]() {base_reason_ready_ = false;}},
    {"base_stale", [this]() {base_receipt_age_ms_ = 250;}},
    {"odometry_stale", [this]() {odometry_receipt_age_ms_ = 250;}},
  };
  for (const auto & defect : defects) {
    SCOPED_TRACE(defect.first);
    odom_x_ = odom_y_ = odom_yaw_ = 0.0;
    base_status_valid_ = base_boot_valid_ = base_sequence_valid_ = true;
    base_ready_ = base_reason_ready_ = true;
    base_receipt_age_ms_ = odometry_receipt_age_ms_ = 0;
    node_->cancel_requested_.store(false);
    defect.second();
    update_evidence();
    auto result = start_dock(stance);
    // Well beyond the 500 ms settling window: no action may have started.
    const bool still_waiting = result.wait_for(900ms) == std::future_status::timeout;
    const auto goals = dispatch_b().goal_count();
    node_->cancel_requested_.store(true);
    ASSERT_EQ(result.wait_for(3s), std::future_status::ready);
    EXPECT_FALSE(result.get());
    EXPECT_TRUE(still_waiting);
    EXPECT_EQ(goals, 0U);
    EXPECT_EQ(dispatch_b().goal_count(), 0U);
    EXPECT_EQ(clear_b().goal_count(), 0U);
    EXPECT_EQ(precise_b().goal_count(), 0U);
  }
  node_->cancel_requested_.store(false);
}

TEST_F(Gate6CenteredDockBehavior, ObserverRequiresContinuousFiveHundredMillisecondsAtBoundary) {
  const auto stance = centered_stance();
  odom_x_ = 0.05;
  update_evidence();
  auto result = start_dock(stance);
  ScopedCleanup cleanup([this]() {node_->cancel_requested_.store(true);});
  ASSERT_EQ(result.wait_for(150ms), std::future_status::timeout);

  // Restore valid stationary evidence at exactly the 0.01 boundary on every axis.
  auto restored = std::chrono::steady_clock::now();
  odom_x_ = 0.01; odom_y_ = -0.01; odom_yaw_ = 0.01;
  update_evidence();
  ASSERT_EQ(result.wait_for(250ms), std::future_status::timeout);
  ASSERT_EQ(dispatch_b().goal_count(), 0U);

  // Interrupt settling, then restore; the 500 ms window must restart.
  odom_y_ = 0.05;
  update_evidence();
  ASSERT_EQ(result.wait_for(100ms), std::future_status::timeout);
  ASSERT_EQ(dispatch_b().goal_count(), 0U);
  restored = std::chrono::steady_clock::now();
  odom_y_ = -0.01;
  update_evidence();

  ASSERT_TRUE(wait_until([this]() {return dispatch_b().goal_count() >= 1U;}, 5s));
  const auto first = dispatch_b().last_goal();
  EXPECT_GE(first.requested_at - restored, 500ms);
  ASSERT_EQ(result.wait_for(10s), std::future_status::ready);
  EXPECT_TRUE(result.get());
}
#endif  // AMR_GATE6_CENTERED_COVERAGE_TESTS

TEST(CenteredBAlignmentAdmission, OnlyResidualsThroughOneCentimeterAdmitDockOnly) {
  using amr_manipulation::CenteredBAlignmentAdmission;
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(0.0),
    CenteredBAlignmentAdmission::DOCK_ONLY);
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(0.009),
    CenteredBAlignmentAdmission::DOCK_ONLY);
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(0.010),
    CenteredBAlignmentAdmission::DOCK_ONLY);
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(0.0100001),
    CenteredBAlignmentAdmission::REJECT);
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(-0.001),
    CenteredBAlignmentAdmission::REJECT);
  EXPECT_EQ(amr_manipulation::centered_b_alignment_admission(
      std::numeric_limits<double>::quiet_NaN()), CenteredBAlignmentAdmission::REJECT);
}

TEST_F(Gate6CenteredDockBehavior, ClearAreaRepairsOnePhysicalPositionMiss) {
  clear_miss_mode_ = 1;
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_TRUE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(clear_accept_count_.load(), 2U);
  EXPECT_EQ(clear_b().goal_count(), 2U);
  EXPECT_EQ(precise_b().goal_count(), 1U);
}

TEST_F(Gate6CenteredDockBehavior, SecondClearAreaMissFailsWithoutDock) {
  clear_miss_mode_ = 2;
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(clear_accept_count_.load(), 2U);
  EXPECT_EQ(clear_b().goal_count(), 2U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, HeadingFailureCannotEnterClearRepair) {
  set_poses_and_reseed(-2.5, 0.0, kPi * 0.5, -2.56, -0.033, kPi * 0.5);
  dispatch_b().set_behavior(NavigateActionHarness::Behavior::ABORT_WITH_FEEDBACK);
  update_evidence();
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 1U);
  EXPECT_EQ(clear_b().goal_count(), 0U);
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, InvalidPhysicalFrameOrMissingCurrentTfFailsBeforeNavigation) {
  physical_frame_valid_ = false;
  update_evidence();
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    const auto wall_now = std::chrono::steady_clock::now();
    ASSERT_TRUE(node_->have_robot_pose_);
    ASSERT_EQ(node_->robot_pose_.header.frame_id, "odom");
    ASSERT_NE(node_->robot_pose_received_, std::chrono::steady_clock::time_point{});
    ASSERT_GE(wall_now, node_->robot_pose_received_);
    ASSERT_LE(wall_now - node_->robot_pose_received_, 200ms);
  }
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 0U);

  physical_frame_valid_ = true;
  tf_defect_ = 1;
  update_evidence();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    const auto wall_now = std::chrono::steady_clock::now();
    ASSERT_TRUE(node_->have_robot_pose_);
    ASSERT_EQ(node_->robot_pose_.header.frame_id, "factory_world");
    ASSERT_NE(node_->robot_pose_received_, std::chrono::steady_clock::time_point{});
    ASSERT_GE(wall_now, node_->robot_pose_received_);
    ASSERT_LE(wall_now - node_->robot_pose_received_, 200ms);
    ASSERT_TRUE(node_->have_base_ && node_->have_odometry_);
    ASSERT_LE(wall_now - node_->base_received_, 200ms);
    ASSERT_LE(wall_now - node_->odometry_received_, 200ms);
    ASSERT_EQ(node_->attachment_states_[1], "attached");
    ASSERT_NE(node_->attachment_state_received_[1], std::chrono::steady_clock::time_point{});
    ASSERT_GE(wall_now, node_->attachment_state_received_[1]);
    ASSERT_LE(wall_now - node_->attachment_state_received_[1], 200ms);
  }
  EXPECT_THROW(
    (void)node_->tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero),
    tf2::TransformException);
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, StalePhysicalEvidenceFailsBeforeNavigation) {
  refresh_physical_ = false;
  update_evidence();
  {
    std::lock_guard<std::mutex> pose_lock(pose_update_mutex_);
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->have_robot_pose_ = true;
    node_->robot_pose_received_ = std::chrono::steady_clock::now() - 201ms;
  }
  update_evidence();
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  amr_manipulation::Product102CenteredDockEvidence dock;
  EXPECT_FALSE(node_->navigate_product102_centered_dock(stance.physical, 10s, dock));
  EXPECT_EQ(dispatch_b().goal_count(), 0U);
}

TEST_F(Gate6CenteredDockBehavior, ActiveGuardCancelsExactGoalOnAttachmentLoss) {
  set_poses(-2.5, 0.1, kPi, -2.56, 0.067, kPi);
  update_evidence();
  dispatch_b().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  update_evidence();
  dispatch_b().set_accepted_hook([this](const auto &) {
    attachment_lost_ = true;
    update_evidence();
  });
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, node_->product_.dispatch_dock,
    node_->product_.dispatch_slots[node_->product_.selected_slot_index]);
  auto result = std::async(std::launch::async, [this, stance]() {
    amr_manipulation::Product102CenteredDockEvidence dock;
    return node_->navigate_product102_centered_dock(stance.physical, 10s, dock);
  });
  ASSERT_TRUE(wait_until([this]() {return dispatch_b().goal_count() == 1U;}, 2s));
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  const auto accepted = dispatch_b().last_goal();
  EXPECT_TRUE(dispatch_b().was_canceled(accepted.uuid));
  EXPECT_EQ(precise_b().goal_count(), 0U);
}

TEST_P(Gate6DispatchBehavior, HeadingCancellationPreventsPrivatePrecision) {
  ASSERT_TRUE(node_->navigate_to({-3.307,-0.023,-3.133},10s)); attach();
  dispatch().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result=std::async(std::launch::async,[this] {
    return node_->navigate_to_aligned_precision({-3.314,0.055,1.67},30s,true);});
  const bool accepted=wait_until([this] {return dispatch().goal_count()==1u;},2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted); ASSERT_EQ(result.wait_for(5s),std::future_status::ready);
  EXPECT_FALSE(result.get()); EXPECT_TRUE(dispatch().was_canceled(dispatch().last_goal().uuid));
  EXPECT_EQ(precise().goal_count(),0u); EXPECT_EQ(precise_server_->goal_count(),0u);
}

TEST_P(Gate6DispatchBehavior, PrecisionCancellationUsesItsActualAcceptedUuid) {
  precise().set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result=std::async(std::launch::async,[this] {
    return node_->navigate_to_dispatch_precise({-3.45,-0.057,std::acos(-1.0)},30s);});
  const bool accepted=wait_until([this] {return precise().goal_count()==1u;},2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted); ASSERT_EQ(result.wait_for(5s),std::future_status::ready);
  EXPECT_FALSE(result.get()); ASSERT_EQ(precise().canceled_count(),1u);
  EXPECT_TRUE(precise().was_canceled(precise().last_goal().uuid));
  EXPECT_EQ(normal_server_->goal_count(),0u); EXPECT_EQ(precise_server_->goal_count(),0u);
}

TEST_P(Gate6DispatchBehavior, PrivateMalformedMissingAndStaleFeedbackFailClosed) {
  for (const auto behavior : {NavigateActionHarness::Behavior::HOLD_MALFORMED_FEEDBACK,
      NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK, NavigateActionHarness::Behavior::HOLD_STALE_FEEDBACK}) {
    precise().set_behavior(behavior);
    EXPECT_FALSE(node_->navigate_to_dispatch_precise({-3.45,-0.057,std::acos(-1.0)},10s));
    EXPECT_TRUE(precise().was_canceled(precise().last_goal().uuid));
  }
  EXPECT_EQ(normal_server_->goal_count(),0u); EXPECT_EQ(precise_server_->goal_count(),0u);
}

TEST_P(Gate6DispatchBehavior, MissingPrivateRouteDoesNotFallbackToPublic) {
  dispatch_precise_servers_[GetParam()-101].reset();
  EXPECT_FALSE(node_->navigate_to_dispatch_precise({-3.45,-0.057,std::acos(-1.0)},10s));
  EXPECT_EQ(normal_server_->goal_count(),0u); EXPECT_EQ(precise_server_->goal_count(),0u);
}

INSTANTIATE_TEST_SUITE_P(Products,Gate6DispatchBehavior,::testing::Values(101,102));

class Gate6ClearApproachBehavior : public Gate6PickupRetreatBehavior {
 protected:
  amr_manipulation::ProductSpec product_spec() override {
    auto product = test_product();
    product.id = 102;
    product.model = "product_b";
    product.dispatch_approach = {-2.5, 0.0, 2.166};
    return product;
  }

  void SetUp() override {
    Gate6PickupRetreatBehavior::SetUp();
    clear_approach_server_->set_accepted_hook([this](const auto &) {
      phase_.store(2);
      update_evidence();
    });
    clear_approach_server_->set_feedback_mutation([](auto & feedback) {
      // Native41 reached XY tolerance with yaw1.722 instead of target2.166.
      feedback.current_pose.pose.orientation.z = std::sin(1.722 / 2.0);
      feedback.current_pose.pose.orientation.w = std::cos(1.722 / 2.0);
    });
    dispatch_servers_[1]->set_accepted_hook([this](const auto &) {
      phase_.store(clear_approach_server_->goal_count() == 0 ? 1 : 3);
      update_evidence();
    });
    set_start_yaw(2.311);
    update_evidence();
    evidence_timer_ = peer_->create_wall_timer(5ms, [this]() {update_evidence();});
  }

  void TearDown() override {
    evidence_timer_.reset();
    Gate6PickupRetreatBehavior::TearDown();
  }

  void set_start_yaw(double yaw) {
    start_yaw_ = yaw;
    nav2_msgs::action::NavigateToPose::Feedback feedback;
    feedback.current_pose.header.frame_id = "map";
    feedback.current_pose.pose.position.x = start_[0];
    feedback.current_pose.pose.position.y = start_[1];
    feedback.current_pose.pose.orientation.z = std::sin(yaw / 2.0);
    feedback.current_pose.pose.orientation.w = std::cos(yaw / 2.0);
    feedback.navigation_time.nanosec = 1000000;
    node_->reset_navigation_feedback();
    node_->record_navigation_feedback(feedback);
  }

  void update_evidence() {
    const int phase = phase_.load();
    const double yaw = phase == 0 ? start_yaw_.load() :
      phase == 2 ? 1.722 : target_[2] + (phase == 3 ? final_yaw_error_.load() : 0.0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.header.stamp = node_->now();
    transform.transform.translation.x = phase < 2 ? start_[0] : target_[0];
    transform.transform.translation.y = phase < 2 ? start_[1] : target_[1];
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->have_robot_pose_ = true;
      node_->robot_pose_received_ = std::chrono::steady_clock::now();
      node_->robot_pose_.pose.position.x = transform.transform.translation.x;
      if (phase >= 2) node_->robot_pose_.pose.position.x += physical_extra_;
      node_->robot_pose_.pose.position.y = transform.transform.translation.y;
      node_->robot_pose_.pose.orientation = transform.transform.rotation;
      node_->attachment_states_[1] = lose_attachment_ && phase >= 2 ? "detached" : "attached";
      node_->attachment_state_received_[1] = std::chrono::steady_clock::now();
    }
    if (phase == 3) {
      transform.transform.translation.x += final_xy_error_;
      switch (tf_defect_.load()) {
        case 1: node_->tf_buffer_.clear(); return;
        case 2: transform.header.stamp = node_->now() - rclcpp::Duration(0, 400000000); break;
        case 3: transform.header.stamp = node_->now() + rclcpp::Duration(0, 100000000); break;
        case 4: transform.header.stamp = builtin_interfaces::msg::Time{}; break;
        case 5:
          transform.transform.rotation.z *= 1.0001;
          transform.transform.rotation.w *= 1.0001;
          break;
        case 6: transform.transform.translation.x = std::numeric_limits<double>::quiet_NaN(); break;
        case 7: transform.transform.rotation.z = std::numeric_limits<double>::quiet_NaN(); break;
      }
      if (tf_defect_ != 0) node_->tf_buffer_.clear();
    }
    (void)node_->tf_buffer_.setTransform(transform, "CA1_behavior", false);
  }

  const std::array<double, 3> start_{-2.439, -0.011, 2.311};
  const std::array<double, 3> target_{-2.472, 0.081, 2.166};
  std::atomic_int phase_{0}, tf_defect_{0};
  std::atomic<double> start_yaw_{2.311}, final_xy_error_{0.0}, final_yaw_error_{0.0};
  std::atomic<double> physical_extra_{0.0};
  std::atomic_bool lose_attachment_{false};
  rclcpp::TimerBase::SharedPtr evidence_timer_;
};

TEST_F(Gate6ClearApproachBehavior, YawDriftRequiresOwnedTerminalHeadingAndCurrentFullPose) {
  ASSERT_TRUE(node_->navigate_product102_clear_approach(target_, 10s));
  ASSERT_EQ(clear_approach_server_->goal_count(), 1U);
  for (std::size_t axis = 0; axis < target_.size(); ++axis)
    EXPECT_NEAR(clear_approach_server_->last_goal().target[axis], target_[axis], 1e-9);
  ASSERT_EQ(dispatch_servers_[1]->goal_count(), 1U);
  const auto heading = dispatch_servers_[1]->last_goal();
  EXPECT_NEAR(heading.target[0], target_[0], 1e-9);
  EXPECT_NEAR(heading.target[1], target_[1], 1e-9);
  EXPECT_NEAR(heading.target[2], target_[2], 1e-9);
  EXPECT_EQ(dispatch_precise_servers_[1]->goal_count(), 0U);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(dispatch_servers_[0]->goal_count(), 0U);
}

TEST_F(Gate6ClearApproachBehavior, InitialHeadingIsRetainedBeforeTranslationAndTerminalTurn) {
  set_start_yaw(0.0);
  ASSERT_TRUE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(clear_approach_server_->goal_count(), 1U);
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 2U);
}

TEST_F(Gate6ClearApproachBehavior, NormalHeadingSuccessCannotRelaxFinalOneCentimeter) {
  final_xy_error_ = 0.011;
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(clear_approach_server_->goal_count(), 1U);
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
}

TEST_F(Gate6ClearApproachBehavior, NormalHeadingSuccessCannotRelaxFinalYaw) {
  final_yaw_error_ = 0.151;
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
}

class Gate6ClearApproachTfBehavior : public Gate6ClearApproachBehavior,
  public ::testing::WithParamInterface<int> {};

TEST_P(Gate6ClearApproachTfBehavior, BadCurrentTfCannotUsePerfectCachedAmclOrFeedback) {
  tf_defect_ = GetParam();
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.pose.position.x = target_[0];
  pose.pose.pose.position.y = target_[1];
  pose.pose.pose.orientation.z = std::sin(target_[2] / 2.0);
  pose.pose.pose.orientation.w = std::cos(target_[2] / 2.0);
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->have_amcl_pose_ = true;
    node_->amcl_pose_ = pose;
    node_->amcl_pose_received_ = std::chrono::steady_clock::now();
  }
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
}
INSTANTIATE_TEST_SUITE_P(CurrentTf, Gate6ClearApproachTfBehavior,
  ::testing::Values(1, 2, 3, 4, 6, 7));

TEST_F(Gate6ClearApproachBehavior, ScaledInputCanonicalizedByTf2NeedsOwnedActionsAndCurrentFullPose) {
  // Real BufferCore lookup canonicalizes this accepted scaled input to unit norm.
  // The production gate validates the returned transform, not the raw fixture input.
  tf_defect_ = 5;
  std::promise<geometry_msgs::msg::TransformStamped> returned_transform;
  auto returned = returned_transform.get_future();
  dispatch_servers_[1]->set_accepted_hook([this, &returned_transform](const auto &) {
    phase_.store(3);
    update_evidence();
    // This hook and the evidence timer share the peer's mutually exclusive group.
    try {
      returned_transform.set_value(
        node_->tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero));
    } catch (...) {
      returned_transform.set_exception(std::current_exception());
    }
  });
  ASSERT_TRUE(node_->navigate_product102_clear_approach(target_, 10s));
  ASSERT_EQ(returned.wait_for(0s), std::future_status::ready);
  geometry_msgs::msg::TransformStamped transform;
  ASSERT_NO_THROW(transform = returned.get());
  const auto & q = transform.transform.rotation;
  EXPECT_TRUE(std::isfinite(q.x) && std::isfinite(q.y) &&
    std::isfinite(q.z) && std::isfinite(q.w));
  EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, 1e-12);
  EXPECT_EQ(transform.header.frame_id, "map");
  EXPECT_EQ(transform.child_frame_id, "base_footprint");
  EXPECT_NEAR(transform.transform.translation.x, target_[0], 1e-9);
  EXPECT_NEAR(transform.transform.translation.y, target_[1], 1e-9);
  EXPECT_NEAR(transform.transform.translation.z, 0.0, 1e-9);
  EXPECT_NEAR(std::atan2(2.0*q.w*q.z, 1.0-2.0*q.z*q.z), target_[2], 1e-9);
  ASSERT_EQ(clear_approach_server_->goal_count(), 1U);
  ASSERT_EQ(dispatch_servers_[1]->goal_count(), 1U);
  for (std::size_t axis = 0; axis < target_.size(); ++axis) {
    EXPECT_NEAR(clear_approach_server_->last_goal().target[axis], target_[axis], 1e-9);
    EXPECT_NEAR(dispatch_servers_[1]->last_goal().target[axis], target_[axis], 1e-9);
  }
  EXPECT_EQ(dispatch_precise_servers_[1]->goal_count(), 0U);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(dispatch_servers_[0]->goal_count(), 0U);
}

class Gate6ClearApproachStageBehavior : public Gate6ClearApproachBehavior,
  public ::testing::WithParamInterface<int> {};

TEST_P(Gate6ClearApproachStageBehavior, CancellationOwnsExactStageUuidAndNoLaterStage) {
  const int stage = GetParam();
  if (stage == 0) set_start_yaw(0.0);
  auto & server = stage == 1 ? *clear_approach_server_ : *dispatch_servers_[1];
  server.set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result = std::async(std::launch::async, [this]() {
    return node_->navigate_product102_clear_approach(target_, 10s);
  });
  const bool accepted = wait_until([&]() {return server.goal_count() == 1U;}, 2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_TRUE(server.was_canceled(server.last_goal().uuid));
  if (stage == 0) EXPECT_EQ(clear_approach_server_->goal_count(), 0U);
  if (stage == 1) EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
  if (stage == 2) EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
}

TEST_P(Gate6ClearApproachStageBehavior, AbortIsNeverSalvagedAndCannotStartLaterStage) {
  const int stage = GetParam();
  if (stage == 0) set_start_yaw(0.0);
  auto & server = stage == 1 ? *clear_approach_server_ : *dispatch_servers_[1];
  server.set_behavior(NavigateActionHarness::Behavior::ABORT_WITH_FEEDBACK);
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  if (stage == 0) EXPECT_EQ(clear_approach_server_->goal_count(), 0U);
  if (stage == 1) EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
  if (stage == 2) EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
}
INSTANTIATE_TEST_SUITE_P(Stages, Gate6ClearApproachStageBehavior, ::testing::Range(0, 3));

TEST_F(Gate6ClearApproachBehavior, LostAttachmentCannotReleaseTranslationOrStartHeading) {
  lose_attachment_ = true;
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
}

TEST_F(Gate6ClearApproachBehavior, EntryAndTranslationBoundsRejectBeforeAnyAction) {
  auto outside = target_;
  outside[0] = start_[0] + 0.151;
  EXPECT_FALSE(node_->navigate_product102_clear_approach(outside, 10s));
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 0s));
  evidence_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->robot_pose_.pose.position.x = -2.7;
  }
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(clear_approach_server_->goal_count(), 0U);
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
}

TEST_F(Gate6ClearApproachBehavior, StalePhysicalEntryCannotStartAnyAction) {
  evidence_timer_.reset();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->robot_pose_received_ -= 1s;
  }
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(clear_approach_server_->goal_count(), 0U);
}

TEST_F(Gate6ClearApproachBehavior, AchievedPhysicalDisplacementCannotExceedExistingBound) {
  physical_extra_ = 0.25;
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 10s));
  EXPECT_EQ(clear_approach_server_->goal_count(), 1U);
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
}

TEST_F(Gate6ClearApproachBehavior, SharedSimulationBudgetExpiresBeforeTerminalHeading) {
  ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  set_start_yaw(0.0);
  dispatch_servers_[1]->set_accepted_hook([this](const auto &) {
    phase_ = 1;
    EXPECT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 140000000000LL), RCL_RET_OK);
    update_evidence();
  });
  clear_approach_server_->set_accepted_hook([this](const auto &) {
    phase_ = 2;
    EXPECT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 221000000000LL), RCL_RET_OK);
    update_evidence();
  });
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 120s));
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 1U);
  EXPECT_EQ(clear_approach_server_->goal_count(), 1U);
}

TEST_F(Gate6ClearApproachBehavior, SimulationRollbackRejectsBeforeTerminalHeading) {
  ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  clear_approach_server_->set_accepted_hook([this](const auto &) {
    phase_ = 2;
    EXPECT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 90000000000LL), RCL_RET_OK);
    update_evidence();
  });
  EXPECT_FALSE(node_->navigate_product102_clear_approach(target_, 120s));
  EXPECT_EQ(dispatch_servers_[1]->goal_count(), 0U);
}

TEST_F(
  Gate6PickupRetreatBehavior,
  NormalAndRegisteredRetreatUseDistinctEndpointsAndExactTargets)
{
  const std::array<double, 3> normal_target{1.10, -2.20, 0.30};
  const std::array<double, 3> retreat_target{3.30, 4.40, -0.70};
  normal_server_->set_behavior(NavigateActionHarness::Behavior::SUCCEED_WITH_FEEDBACK);
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::SUCCEED_WITH_FEEDBACK);

  ASSERT_TRUE(node_->navigate_to(normal_target, 10s));
  ASSERT_TRUE(node_->navigate_to_registered_retreat(retreat_target, 10s));

  ASSERT_EQ(normal_server_->goal_count(), 1U);
  ASSERT_EQ(retreat_server_->goal_count(), 1U);
  const auto normal_goal = normal_server_->last_goal();
  const auto retreat_goal = retreat_server_->last_goal();
  EXPECT_EQ(normal_goal.target, normal_target);
  EXPECT_EQ(retreat_goal.target, retreat_target);
  EXPECT_EQ(normal_goal.frame_id, "map");
  EXPECT_EQ(retreat_goal.frame_id, "map");
  EXPECT_EQ(normal_server_->canceled_count(), 0U);
  EXPECT_EQ(retreat_server_->canceled_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, PlacementPrecisionUsesExistingEndpointAndExactTarget)
{
  const std::array<double, 3> target{-3.450904, -0.057, std::acos(-1.0)};
  ASSERT_TRUE(node_->navigate_to_precise(target, 10s));
  ASSERT_EQ(precise_server_->goal_count(), 1U);
  EXPECT_EQ(precise_server_->last_goal().target, target);
  EXPECT_EQ(precise_server_->last_goal().frame_id, "map");
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, StageStartRequiresFreshGenuineForwardedBoundary)
{
  auto publisher = peer_->create_publisher<amr_interfaces::msg::ManipulatorStatus>(
    "/amr/manipulation/status", amr_interfaces::qos::authority());
  auto valid = std::make_shared<std::atomic_bool>(false);
  auto timer = peer_->create_wall_timer(10ms, [this, publisher, valid]() {
    amr_interfaces::msg::ManipulatorStatus status;
    status.header.stamp = node_->now();
    status.source_boot_id = node_->boot_id_ == 1U ? 2U : 1U;
    status.sequence = 1U;
    status.valid = true;
    status.state = amr_interfaces::msg::ManipulatorStatus::STARTING;
    status.detail = valid->load() ? "Gate 6 mass stage is starting" : "preparation is starting";
    publisher->publish(status);
  });
  EXPECT_FALSE(node_->wait_for_stage_start_forwarded(200ms));
  valid->store(true);
  EXPECT_TRUE(node_->wait_for_stage_start_forwarded(2s));
  timer->cancel();
}

TEST_F(Gate6PickupRetreatBehavior, StageStartRejectsMissingBoundaryAndCancellation)
{
  EXPECT_FALSE(node_->wait_for_stage_start_forwarded(100ms));
  auto waiting = std::async(std::launch::async, [this]() {
    return node_->wait_for_stage_start_forwarded(2s);
  });
  node_->cancel_requested_.store(true);
  node_->terminal_evidence_condition_.notify_all();
  ASSERT_EQ(waiting.wait_for(500ms), std::future_status::ready);
  EXPECT_FALSE(waiting.get());
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceBacksAwayWithoutNormalTurn)
{
  const auto evidence = dispatch_evidence();
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  ASSERT_TRUE(node_->navigate_empty_dispatch_clearance(10s));
  ASSERT_EQ(retreat_server_->goal_count(), 1U);
  const auto target = retreat_server_->last_goal().target;
  EXPECT_NEAR(target[0], -2.44, 1e-9);
  EXPECT_NEAR(target[1], -0.09, 1e-9);
  EXPECT_NEAR(target[2], std::acos(-1.0), 1e-9);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(precise_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceRejectsMissingLocalization)
{
  const auto evidence = dispatch_evidence(false);
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  EXPECT_FALSE(node_->navigate_empty_dispatch_clearance(10s));
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceRejectsAttachedOrMovingState)
{
  const auto evidence = dispatch_evidence();
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  node_->set_status(amr_interfaces::msg::ManipulatorStatus::STOWED_LOADED, true, true, "loaded");
  EXPECT_FALSE(node_->navigate_empty_dispatch_clearance(10s));
  node_->set_status(amr_interfaces::msg::ManipulatorStatus::STOWED_EMPTY, true, false, "empty");
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
  }
  EXPECT_FALSE(node_->navigate_empty_dispatch_clearance(10s));
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceRejectsStaleLocalization)
{
  const auto evidence = dispatch_evidence(false);
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_footprint";
  transform.transform.translation.x = -3.54;
  transform.transform.rotation.z = 1.0;
  node_->tf_buffer_.setTransform(transform, "clearance_test", false);
  EXPECT_FALSE(node_->navigate_empty_dispatch_clearance(10s));
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceLostDetachCancelsOwnedGoal)
{
  const auto evidence = dispatch_evidence();
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result = std::async(std::launch::async, [this]() {
    return node_->navigate_empty_dispatch_clearance(30s);
  });
  ASSERT_TRUE(wait_until([this]() { return retreat_server_->goal_count() == 1U; }, 2s));
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
  }
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_TRUE(retreat_server_->was_canceled(retreat_server_->last_goal().uuid));
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceRejectsDisplacedFinalProduct)
{
  const auto evidence = dispatch_evidence(true, true);
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  EXPECT_FALSE(node_->navigate_empty_dispatch_clearance(10s));
  EXPECT_EQ(retreat_server_->goal_count(), 1U);
}

TEST_F(Gate6PickupRetreatBehavior, EmptyDispatchClearanceCancellationOwnsRetreatTerminal)
{
  const auto evidence = dispatch_evidence();
  ASSERT_TRUE(wait_until([this]() {
    geometry_msgs::msg::PoseStamped pose;
    return node_->latest_robot_pose(pose);
  }, 2s));
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result = std::async(std::launch::async, [this]() {
    return node_->navigate_empty_dispatch_clearance(30s);
  });
  ASSERT_TRUE(wait_until([this]() { return retreat_server_->goal_count() == 1U; }, 2s));
  node_->cancel_requested_.store(true);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  ASSERT_EQ(retreat_server_->canceled_count(), 1U);
  EXPECT_TRUE(retreat_server_->was_canceled(retreat_server_->last_goal().uuid));
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, LateralPrecisionAlignsInPlaceBeforeTravel)
{
  const std::array<double, 3> start{-3.307, -0.023, -3.133};
  const std::array<double, 3> target{-3.314, 0.055, 1.67};
  ASSERT_TRUE(node_->navigate_to(start, 10s));
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  ASSERT_TRUE(node_->navigate_to_aligned_precision(target, 10s));
  ASSERT_EQ(normal_server_->goal_count(), 2U);
  EXPECT_EQ(normal_server_->last_goal().target,
    (std::array<double, 3>{start[0], start[1], target[2]}));
  ASSERT_EQ(precise_server_->goal_count(), 1U);
  EXPECT_EQ(precise_server_->last_goal().target, target);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, AlignedPrecisionSkipsUnnecessaryHeadingGoal)
{
  const std::array<double, 3> start{-3.312, -0.021, -3.100};
  const std::array<double, 3> target{-3.387, -0.030, -3.050};
  ASSERT_TRUE(node_->navigate_to(start, 10s));
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  ASSERT_TRUE(node_->navigate_to_aligned_precision(target, 10s));
  EXPECT_EQ(normal_server_->goal_count(), 1U);
  ASSERT_EQ(precise_server_->goal_count(), 1U);
  EXPECT_EQ(precise_server_->last_goal().target, target);
}

TEST_F(Gate6PickupRetreatBehavior, HeadingCancellationPreventsPrecisionTravel)
{
  ASSERT_TRUE(node_->navigate_to({-3.307, -0.023, -3.133}, 10s));
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  normal_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result = std::async(std::launch::async, [this]() {
    return node_->navigate_to_aligned_precision({-3.314, 0.055, 1.67}, 30s);
  });
  const bool accepted = wait_until([this]() { return normal_server_->goal_count() == 2U; }, 2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  EXPECT_TRUE(normal_server_->was_canceled(normal_server_->last_goal().uuid));
  EXPECT_EQ(precise_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, PrecisionCancellationUsesAcceptedUuid)
{
  const std::array<double, 3> target{-3.450904, -0.057, std::acos(-1.0)};
  precise_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  auto result = std::async(std::launch::async, [this, target]() {
    return node_->navigate_to_precise(target, 30s);
  });
  const bool accepted = wait_until([this]() { return precise_server_->goal_count() == 1U; }, 2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  ASSERT_EQ(precise_server_->canceled_count(), 1U);
  EXPECT_TRUE(precise_server_->was_canceled(precise_server_->last_goal().uuid));
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, MalformedFeedbackDeniesPrecision)
{
  precise_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_MALFORMED_FEEDBACK);
  EXPECT_FALSE(node_->navigate_to_precise({-3.450904, -0.057, std::acos(-1.0)}, 10s));
  ASSERT_EQ(precise_server_->goal_count(), 1U);
  EXPECT_EQ(precise_server_->canceled_count(), 1U);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, RetreatCancellationUsesAcceptedUuidOnRetreatServer)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);
  node_->cancel_requested_.store(false);

  auto result = std::async(
    std::launch::async,
    [this, target]() {
      return node_->navigate_to_registered_retreat(target, 30s);
    });
  const bool accepted = wait_until([this]() { return retreat_server_->goal_count() == 1U; }, 2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());

  const auto accepted_goal = retreat_server_->last_goal();
  ASSERT_EQ(retreat_server_->canceled_count(), 1U);
  EXPECT_TRUE(retreat_server_->was_canceled(accepted_goal.uuid));
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, RejectedAbortedAndCanceledResultsFailClosed)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};

  normal_server_->set_behavior(NavigateActionHarness::Behavior::REJECT);
  EXPECT_FALSE(node_->navigate_to(target, 10s));
  node_->cancel_requested_.store(false);

  normal_server_->set_behavior(NavigateActionHarness::Behavior::ABORT_WITH_FEEDBACK);
  EXPECT_FALSE(node_->navigate_to(target, 10s));
  node_->cancel_requested_.store(false);

  normal_server_->set_behavior(NavigateActionHarness::Behavior::CANCELED_WITH_FEEDBACK);
  auto result = std::async(
    std::launch::async,
    [this, target]() {
      return node_->navigate_to(target, 30s);
    });
  const bool accepted = wait_until([this]() { return normal_server_->goal_count() == 2U; }, 2s);
  node_->cancel_requested_.store(true);
  ASSERT_TRUE(accepted);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_FALSE(result.get());
  node_->cancel_requested_.store(false);
}

TEST_F(Gate6PickupRetreatBehavior, MalformedFeedbackDeniesRetreat)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_MALFORMED_FEEDBACK);

  EXPECT_FALSE(node_->navigate_to_registered_retreat(target, 10s));
  ASSERT_EQ(retreat_server_->goal_count(), 1U);
  EXPECT_EQ(retreat_server_->canceled_count(), 1U);
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    EXPECT_TRUE(node_->navigation_feedback_received_);
    EXPECT_TRUE(node_->navigation_feedback_invalid_);
  }
}

TEST_F(Gate6PickupRetreatBehavior, MissingFeedbackDeniesNormal)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};
  normal_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_NO_FEEDBACK);

  EXPECT_FALSE(node_->navigate_to(target, 10s));
  ASSERT_EQ(normal_server_->goal_count(), 1U);
  EXPECT_EQ(normal_server_->canceled_count(), 1U);
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    EXPECT_FALSE(node_->navigation_feedback_received_);
  }
}

TEST_F(Gate6PickupRetreatBehavior, StaleFeedbackDeniesRetreat)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};
  retreat_server_->set_behavior(NavigateActionHarness::Behavior::HOLD_STALE_FEEDBACK);

  EXPECT_FALSE(node_->navigate_to_registered_retreat(target, 10s));
  ASSERT_EQ(retreat_server_->goal_count(), 1U);
  EXPECT_EQ(retreat_server_->canceled_count(), 1U);
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    EXPECT_TRUE(node_->navigation_feedback_received_);
    EXPECT_FALSE(node_->navigation_feedback_invalid_);
  }
}

TEST_F(Gate6PickupRetreatBehavior, SuccessWithoutFeedbackRequiresFreshValidAmcl)
{
  const std::array<double, 3> target{1.50, 3.00, 0.0};
  normal_server_->set_behavior(NavigateActionHarness::Behavior::SUCCEED_WITHOUT_FEEDBACK);

  EXPECT_FALSE(node_->navigate_to(target, 10s));
  node_->cancel_requested_.store(false);

  const auto before_odom_receipt = amcl_receipt_time();
  publish_amcl(target, "odom");
  ASSERT_TRUE(wait_for_amcl_receipt(before_odom_receipt, target, "odom"));
  EXPECT_FALSE(node_->navigate_to(target, 10s));
  node_->cancel_requested_.store(false);

  const auto before_map_receipt = amcl_receipt_time();
  publish_amcl(target);
  ASSERT_TRUE(wait_for_amcl_receipt(before_map_receipt, target, "map"));
  EXPECT_TRUE(node_->navigate_to(target, 10s));
}

TEST_F(Gate6PickupRetreatBehavior, SuccessfulRetreatWithLostAttachmentCannotAdmitDispatch)
{
  const auto product = test_product();
  auto state_pub = peer_->create_publisher<std_msgs::msg::String>(
    amr_manipulation::MassStageNode::attachment_topic(product.id, "state"),
    amr_interfaces::qos::state());
  std_msgs::msg::String state;
  state.data = "attached";
  ASSERT_TRUE(wait_until(
    [&]() {
      state_pub->publish(state);
      return node_->native_attachment_state_is("attached");
    }, 2s));

  retreat_server_->set_behavior(NavigateActionHarness::Behavior::SUCCEED_WITH_FEEDBACK);
  ASSERT_TRUE(node_->navigate_to_registered_retreat(product.pickup_station, 10s));
  ASSERT_EQ(retreat_server_->goal_count(), 1U);

  const auto before_positive_amcl_receipt = amcl_receipt_time();
  publish_amcl(product.pickup_station);
  ASSERT_TRUE(wait_for_amcl_receipt(
    before_positive_amcl_receipt, product.pickup_station, "map"));

  geometry_msgs::msg::PoseStamped pickup_station_achieved;
  geometry_msgs::msg::PoseStamped dispatch_translation_start;
  ASSERT_TRUE(amr_manipulation::pickup_station_admission_proof(
    node_, product, true, pickup_station_achieved, dispatch_translation_start));
  EXPECT_EQ(normal_server_->goal_count(), 0U);

  state.data = "detached";
  ASSERT_TRUE(wait_until(
    [&]() {
      state_pub->publish(state);
      return node_->native_attachment_state_is("detached");
    }, 2s));

  const auto before_negative_amcl_receipt = amcl_receipt_time();
  publish_amcl(product.pickup_station);
  ASSERT_TRUE(wait_for_amcl_receipt(
    before_negative_amcl_receipt, product.pickup_station, "map"));

  const bool admitted_after_detachment = amr_manipulation::pickup_station_admission_proof(
    node_, product, true, pickup_station_achieved, dispatch_translation_start);
  EXPECT_FALSE(admitted_after_detachment);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, AttachmentAdmissionEvidenceIsFailClosed)
{
  EXPECT_FALSE(node_->native_attachment_state_is("attached"));
  auto state_pub = peer_->create_publisher<std_msgs::msg::String>(
    amr_manipulation::MassStageNode::attachment_topic(101, "state"),
    amr_interfaces::qos::state());
  std_msgs::msg::String state;
  state.data = "detached";
  ASSERT_TRUE(wait_until(
    [&]() {
      state_pub->publish(state);
      return node_->native_attachment_state_is("detached");
    }, 2s));
  EXPECT_FALSE(node_->native_attachment_state_is("attached"));

  state.data = "attached";
  ASSERT_TRUE(wait_until(
    [&]() {
      state_pub->publish(state);
      return node_->native_attachment_state_is("attached");
    }, 2s));
  EXPECT_TRUE(node_->native_attachment_state_is("attached"));

  state.data = "unknown";
  ASSERT_TRUE(wait_until(
    [&]() {
      state_pub->publish(state);
      return node_->native_attachment_state_is("unknown");
    }, 2s));
  EXPECT_FALSE(node_->native_attachment_state_is("attached"));
}

TEST_F(Gate6PickupRetreatBehavior, AdmissionWaitsForDelayedPostTurnAmclSample)
{
  const auto product = test_product();
  auto state_pub = peer_->create_publisher<std_msgs::msg::String>(
    amr_manipulation::MassStageNode::attachment_topic(product.id, "state"),
    amr_interfaces::qos::state());
  std_msgs::msg::String state;
  state.data = "attached";
  ASSERT_TRUE(wait_until([&]() {
    state_pub->publish(state);
    return node_->native_attachment_state_is("attached");
  }, 2s));
  auto old_pose = product.pickup_station;
  old_pose[2] = -0.2048203671;  // Last AMCL yaw at run12 admission.
  const auto previous_receipt = amcl_receipt_time();
  publish_amcl(old_pose);
  ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, old_pose, "map"));
  geometry_msgs::msg::PoseStamped achieved;
  geometry_msgs::msg::PoseStamped dispatch_start;
  auto admission = std::async(std::launch::async, [&]() {
    return amr_manipulation::pickup_station_admission_proof(
      node_, product, true, achieved, dispatch_start);
  });
  EXPECT_EQ(admission.wait_for(40ms), std::future_status::timeout);
  auto terminal_pose = product.pickup_station;
  terminal_pose[2] = -0.141198;  // Next AMCL yaw, after the failed run12 check.
  publish_amcl(terminal_pose);
  ASSERT_EQ(admission.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(admission.get());
  EXPECT_NEAR(dispatch_start.pose.orientation.z, std::sin(terminal_pose[2] * 0.5), 1e-9);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, AdmissionRequestsPostTurnAmclObservation)
{
  const auto product = test_product();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  auto old_pose = product.pickup_station;
  old_pose[0] += 0.0185478075;
  old_pose[2] = -0.161558688;  // Run37's last, out-of-yaw AMCL observation.
  const auto previous_receipt = amcl_receipt_time();
  publish_amcl(old_pose);
  ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, old_pose, "map"));
  std::atomic_uint requests{0};
  auto service = peer_->create_service<std_srvs::srv::Empty>(
    "/amr/request_nomotion_update",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
      ++requests;
      auto updated = product.pickup_station;
      updated[2] = -0.149;
      publish_amcl(updated);  // No publication occurs without the real request.
    });
  auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
  ASSERT_TRUE(discovery->wait_for_service(2s));
  ASSERT_TRUE(node_->wait_for_amcl_terminal_pose(product.pickup_station, 1s));
  EXPECT_EQ(requests.load(), 1U);
  geometry_msgs::msg::PoseStamped proof;
  ASSERT_TRUE(node_->latest_navigation_feedback_pose(proof));
  EXPECT_NEAR(proof.pose.orientation.z, std::sin(-0.149 * 0.5), 1e-9);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, ValidCachedAdmissionDoesNotRequestAmclUpdate)
{
  const auto product = test_product();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  const auto previous_receipt = amcl_receipt_time();
  publish_amcl(product.pickup_station);
  ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, product.pickup_station, "map"));
  std::atomic_uint requests{0};
  auto service = peer_->create_service<std_srvs::srv::Empty>(
    "/amr/request_nomotion_update",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {++requests;});
  auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
  ASSERT_TRUE(discovery->wait_for_service(2s));
  EXPECT_TRUE(node_->wait_for_amcl_terminal_pose(product.pickup_station));
  EXPECT_FALSE(wait_until([&]() {return requests.load() != 0;}, 100ms));
  EXPECT_EQ(requests.load(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, AmclUpdateReplyAloneCannotGrantProofOrExtendDeadline)
{
  const auto product = test_product();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  auto old_pose = product.pickup_station;
  old_pose[2] = -0.161558688;
  const auto previous_receipt = amcl_receipt_time();
  publish_amcl(old_pose);
  ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, old_pose, "map"));
  std::atomic_uint requests{0};
  auto service = peer_->create_service<std_srvs::srv::Empty>(
    "/amr/request_nomotion_update",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {++requests;});
  auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
  ASSERT_TRUE(discovery->wait_for_service(2s));
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(node_->wait_for_amcl_terminal_pose(product.pickup_station, 120ms));
  EXPECT_GE(std::chrono::steady_clock::now() - started, 120ms);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
  EXPECT_EQ(requests.load(), 1U);
  EXPECT_FALSE(node_->navigation_feedback_received_);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, RequestedAmclUpdatePreservesPoseAndAcquisitionGates)
{
  const auto product = test_product();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  for (int defect = 0; defect < 11; ++defect) {
    SCOPED_TRACE(defect);
    node_->reset_navigation_feedback();
    auto old_pose = product.pickup_station;
    old_pose[2] = -0.161558688;
    const auto previous_receipt = amcl_receipt_time();
    publish_amcl(old_pose);
    ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, old_pose, "map"));
    builtin_interfaces::msg::Time old_stamp;
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      old_stamp = node_->amcl_pose_.header.stamp;
    }
    std::atomic_uint requests{0};
    auto service = peer_->create_service<std_srvs::srv::Empty>(
      "/amr/request_nomotion_update",
      [&, defect, old_stamp](const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        ++requests;
        geometry_msgs::msg::PoseWithCovarianceStamped message;
        message.header.frame_id = "map";
        message.header.stamp = peer_->now();
        message.pose.pose.position.x = product.pickup_station[0];
        message.pose.pose.position.y = product.pickup_station[1];
        message.pose.pose.orientation.w = 1.0;
        if (defect == 0) message.header.frame_id = "odom";
        if (defect == 1) message.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        if (defect == 2) message.pose.pose.position.x += 0.071;
        if (defect == 3) {
          message.pose.pose.orientation.z = std::sin(0.151 * 0.5);
          message.pose.pose.orientation.w = std::cos(0.151 * 0.5);
        }
        if (defect == 4) message.header.stamp = old_stamp;  // New receipt, duplicate acquisition.
        if (defect == 5) message.header.stamp = rclcpp::Time(old_stamp) + rclcpp::Duration(0, 1);
        if (defect == 6) message.header.stamp = builtin_interfaces::msg::Time{};
        if (defect == 7) {message.header.stamp.sec = -1; message.header.stamp.nanosec = 0;}
        if (defect == 8) message.header.stamp.nanosec = 1000000000;
        if (defect == 9) message.header.stamp = peer_->now() + rclcpp::Duration(10, 0);
        if (defect == 10) message.header.stamp = rclcpp::Time(old_stamp) - rclcpp::Duration(0, 1);
        amcl_pub_->publish(message);
      });
    auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
    ASSERT_TRUE(discovery->wait_for_service(2s));
    EXPECT_FALSE(node_->wait_for_amcl_terminal_pose(product.pickup_station, 120ms));
    EXPECT_EQ(requests.load(), 1U);
    EXPECT_FALSE(node_->navigation_feedback_received_);
  }
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, PendingAmclRequestIsDiscardedBeforeLateResponse)
{
  const auto product = test_product();
  for (int stop = 0; stop < 3; ++stop) {
    SCOPED_TRACE(stop);
    node_->cancel_requested_.store(false);
    node_->reset_navigation_feedback();
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->attachment_states_[0] = "attached";
      node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
    }
    auto old_pose = product.pickup_station;
    old_pose[2] = -0.161558688;
    const auto previous_receipt = amcl_receipt_time();
    publish_amcl(old_pose);
    ASSERT_TRUE(wait_for_amcl_receipt(previous_receipt, old_pose, "map"));
    std::promise<void> entered, release_response;
    auto entered_future = entered.get_future();
    auto release = release_response.get_future().share();
    std::atomic_uint requests{0};
    auto service = peer_->create_service<std_srvs::srv::Empty>(
      "/amr/request_nomotion_update",
      [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        if (++requests == 1U) entered.set_value();
        release.wait();
        publish_amcl(product.pickup_station);
      });
    auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
    ASSERT_TRUE(discovery->wait_for_service(2s));
    const auto started = std::chrono::steady_clock::now();
    auto admission = std::async(std::launch::async, [&]() {
      return node_->wait_for_amcl_terminal_pose(product.pickup_station, stop == 2 ? 120ms : 2s);
    });
    ASSERT_EQ(entered_future.wait_for(1s), std::future_status::ready);
    if (stop == 0) node_->cancel_requested_.store(true);
    if (stop == 1) {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->attachment_states_[0] = "detached";
    }
    node_->terminal_evidence_condition_.notify_all();
    ASSERT_EQ(admission.wait_for(300ms), std::future_status::ready);
    EXPECT_FALSE(admission.get());
    EXPECT_LT(std::chrono::steady_clock::now() - started, 500ms);
    if (stop == 2) EXPECT_GE(std::chrono::steady_clock::now() - started, 120ms);
    const auto before_late_pose = amcl_receipt_time();
    release_response.set_value();
    ASSERT_TRUE(wait_for_amcl_receipt(before_late_pose, product.pickup_station, "map"));
    EXPECT_EQ(requests.load(), 1U);
    EXPECT_FALSE(node_->navigation_feedback_received_);  // Late pose/reply grants no permission.
  }
  EXPECT_EQ(normal_server_->goal_count(), 0U);
  EXPECT_EQ(retreat_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, ClockRollbackInterruptsAmclObservationRequest)
{
  const auto product = test_product();
  ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 100000000000LL),
    RCL_RET_OK);
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  std::atomic_uint requests{0};
  auto service = peer_->create_service<std_srvs::srv::Empty>(
    "/amr/request_nomotion_update",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {++requests;});
  auto discovery = peer_->create_client<std_srvs::srv::Empty>("/amr/request_nomotion_update");
  ASSERT_TRUE(discovery->wait_for_service(2s));
  auto admission = std::async(std::launch::async, [&]() {
    return node_->wait_for_amcl_terminal_pose(product.pickup_station, 2s);
  });
  ASSERT_TRUE(wait_until([&]() {return requests.load() == 1U;}, 1s));
  ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), 99000000000LL),
    RCL_RET_OK);
  node_->terminal_evidence_condition_.notify_all();
  ASSERT_EQ(admission.wait_for(200ms), std::future_status::ready);
  EXPECT_FALSE(admission.get());
  EXPECT_FALSE(node_->navigation_feedback_received_);
  EXPECT_EQ(requests.load(), 1U);
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, TerminalPoseWaitRejectsInvalidEvidenceWithinDeadline)
{
  const auto product = test_product();
  {
    std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
    node_->attachment_states_[0] = "attached";
    node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
  }
  for (int invalid_case = 0; invalid_case < 6; ++invalid_case) {
    SCOPED_TRACE(invalid_case);
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->have_amcl_pose_ = invalid_case != 0;
      node_->amcl_pose_.header.frame_id = invalid_case == 1 ? "odom" : "map";
      node_->amcl_pose_.pose.pose.position.x = product.pickup_station[0];
      node_->amcl_pose_.pose.pose.position.y = product.pickup_station[1];
      node_->amcl_pose_.pose.pose.orientation.z = 0.0;
      node_->amcl_pose_.pose.pose.orientation.w = 1.0;
      node_->amcl_pose_received_ = std::chrono::steady_clock::now();
      if (invalid_case == 2) node_->amcl_pose_received_ -= 7s;
      if (invalid_case == 3) {
        node_->amcl_pose_.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
      }
      if (invalid_case == 4) node_->amcl_pose_.pose.pose.position.x += 0.071;
      if (invalid_case == 5) {
        node_->amcl_pose_.pose.pose.orientation.z = std::sin(0.151 * 0.5);
        node_->amcl_pose_.pose.pose.orientation.w = std::cos(0.151 * 0.5);
      }
    }
    const auto started = std::chrono::steady_clock::now();
    EXPECT_FALSE(node_->wait_for_amcl_terminal_pose(product.pickup_station, 100ms));
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(elapsed, 100ms);
    EXPECT_LT(elapsed, 500ms);
  }
  EXPECT_EQ(normal_server_->goal_count(), 0U);
}

TEST_F(Gate6PickupRetreatBehavior, CancellationAndAttachmentLossInterruptTerminalPoseWait)
{
  const auto product = test_product();
  for (bool cancel : {true, false}) {
    SCOPED_TRACE(cancel);
    node_->cancel_requested_.store(false);
    {
      std::lock_guard<std::mutex> lock(node_->evidence_mutex_);
      node_->attachment_states_[0] = "attached";
      node_->attachment_state_received_[0] = std::chrono::steady_clock::now();
      node_->have_amcl_pose_ = false;
    }
    auto result = std::async(std::launch::async, [&]() {
      return node_->wait_for_amcl_terminal_pose(product.pickup_station);
    });
    EXPECT_EQ(result.wait_for(40ms), std::future_status::timeout);
    if (cancel) {
      auto client = peer_->create_client<std_srvs::srv::Trigger>(product.cancel_service);
      ASSERT_TRUE(client->wait_for_service(1s));
      auto response = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      ASSERT_EQ(response.wait_for(1s), std::future_status::ready);
      EXPECT_TRUE(response.get()->success);
    } else {
      auto publisher = peer_->create_publisher<std_msgs::msg::String>(
        amr_manipulation::MassStageNode::attachment_topic(product.id, "state"),
        amr_interfaces::qos::state());
      std_msgs::msg::String state;
      state.data = "detached";
      ASSERT_TRUE(wait_until([&]() {
        publisher->publish(state);
        return node_->native_attachment_state_is("detached");
      }, 1s));
    }
    ASSERT_EQ(result.wait_for(200ms), std::future_status::ready);
    EXPECT_FALSE(result.get());
    EXPECT_EQ(normal_server_->goal_count(), 0U);
    EXPECT_EQ(retreat_server_->goal_count(), 0U);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
