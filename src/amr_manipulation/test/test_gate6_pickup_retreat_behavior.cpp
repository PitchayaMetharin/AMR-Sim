#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "amr_interfaces/qos_profiles.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#define private public
#define main gate6_mass_stage_test_entrypoint
#include "../src/gate6_mass_stage.cpp"
#undef main
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
  };

  NavigateActionHarness(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & endpoint)
  : endpoint_(endpoint)
  {
    server_ = rclcpp_action::create_server<Action>(
      node, endpoint_,
      [this](
        const GoalUUID & uuid,
        std::shared_ptr<const Action::Goal> goal) {
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
        const auto publish_feedback = [goal, behavior]() {
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
          } else if (behavior == Behavior::ABORT_WITH_FEEDBACK) {
            goal->abort(result);
          }
          return;
        }

        if (behavior == Behavior::SUCCEED_WITHOUT_FEEDBACK) {
          goal->succeed(std::make_shared<Action::Result>());
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
          }
        });
      });
  }

  ~NavigateActionHarness()
  {
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

 private:
  std::string endpoint_;
  std::atomic<Behavior> behavior_{Behavior::SUCCEED_WITH_FEEDBACK};
  std::atomic_bool shutting_down_{false};
  mutable std::mutex mutex_;
  std::vector<GoalObservation> goals_;
  std::vector<GoalUUID> canceled_goals_;
  std::vector<std::thread> workers_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

class Gate6PickupRetreatBehavior : public ::testing::Test {
 protected:
  void SetUp() override
  {
    static std::atomic_uint node_index{0};
    const auto index = node_index.fetch_add(1);
    peer_ = std::make_shared<rclcpp::Node>(
      "gate6_pickup_retreat_peer_" + std::to_string(index));
    amcl_pub_ = peer_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amr/amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    normal_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose");
    retreat_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose_retreat");
    precise_server_ = std::make_unique<NavigateActionHarness>(
      peer_, "/amr/mission/navigate_to_pose_precise");
    node_ = std::make_shared<amr_manipulation::MassStageNode>(
      test_product(), rclcpp::NodeOptions());
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(peer_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    ASSERT_TRUE(node_->navigation_client_->wait_for_action_server(2s));
    ASSERT_TRUE(node_->retreat_navigation_client_->wait_for_action_server(2s));
    ASSERT_TRUE(node_->precise_navigation_client_->wait_for_action_server(2s));
  }

  void TearDown() override
  {
    if (node_) {
      node_->cancel_requested_.store(true);
    }
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    normal_server_.reset();
    retreat_server_.reset();
    precise_server_.reset();
    node_.reset();
    amcl_pub_.reset();
    peer_.reset();
  }

  void publish_amcl(
    const std::array<double, 3> & target,
    const std::string & frame_id = "map",
    bool finite = true)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = frame_id;
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
  std::shared_ptr<amr_manipulation::MassStageNode> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pub_;
  std::unique_ptr<NavigateActionHarness> normal_server_;
  std::unique_ptr<NavigateActionHarness> retreat_server_;
  std::unique_ptr<NavigateActionHarness> precise_server_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};

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
