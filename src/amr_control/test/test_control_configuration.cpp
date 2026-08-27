#include <chrono>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "amr_interfaces/qos_profiles.hpp"

#define main command_arbitration_main
#include "../src/command_arbitration_node.cpp"
#undef main

using namespace std::chrono_literals;

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
  return predicate();
}

TEST(ControlConfiguration, ArbitrationRejectsInvalidNumericParameters) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("source_timeout_ms", 0),
      rclcpp::Parameter("manipulator_status_timeout_ms", 0),
      rclcpp::Parameter("output_frequency", 0.0),
      rclcpp::Parameter("max_linear_velocity", -1.0),
      rclcpp::Parameter(
          "max_angular_acceleration",
          std::numeric_limits<double>::quiet_NaN()),
  });
  auto node =
      std::make_shared<amr_control::CommandArbitrationNode>(options);
  const auto state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
      state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ControlBehavior, ArbitrationExpiresTheSourceCommandToZero) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("source_timeout_ms", 80),
      rclcpp::Parameter("output_frequency", 50.0),
      rclcpp::Parameter("max_linear_acceleration", 100.0),
      rclcpp::Parameter("max_angular_acceleration", 100.0),
  });
  auto arbitration =
      std::make_shared<amr_control::CommandArbitrationNode>(options);
  ASSERT_EQ(
      arbitration->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
      arbitration->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto peer = std::make_shared<rclcpp::Node>("arbitration_behavior_peer");
  auto input = peer->create_publisher<geometry_msgs::msg::Twist>(
      "/amr/mpc/cmd_vel", amr_interfaces::qos::nav2_command_input());
  bool saw_nonzero = false;
  bool saw_zero_after_nonzero = false;
  auto output = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/amr/control/cmd_vel", amr_interfaces::qos::command(),
      [&](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        if (message->twist.linear.x != 0.0) {
          saw_nonzero = true;
        } else if (saw_nonzero) {
          saw_zero_after_nonzero = true;
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arbitration->get_node_base_interface());
  executor.add_node(peer);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.25;
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        input->publish(command);
        return saw_nonzero;
      },
      2s));
  EXPECT_TRUE(spin_until(
      executor, [&]() { return saw_zero_after_nonzero; }, 500ms));

  executor.remove_node(peer);
  executor.remove_node(arbitration->get_node_base_interface());
}

TEST(ControlBehavior, ManipulatorInterlockIsFailClosedAndRecoverable) {
  using ManipulatorStatus = amr_interfaces::msg::ManipulatorStatus;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("source_timeout_ms", 200),
      rclcpp::Parameter("require_manipulator_stowed", true),
      rclcpp::Parameter("manipulator_status_timeout_ms", 80),
      rclcpp::Parameter("output_frequency", 100.0),
      rclcpp::Parameter("max_linear_acceleration", 100.0),
      rclcpp::Parameter("max_angular_acceleration", 100.0),
  });
  auto arbitration =
      std::make_shared<amr_control::CommandArbitrationNode>(options);
  ASSERT_EQ(
      arbitration->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
      arbitration->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto peer = std::make_shared<rclcpp::Node>("manipulator_interlock_peer");
  auto input = peer->create_publisher<geometry_msgs::msg::Twist>(
      "/amr/mpc/cmd_vel", amr_interfaces::qos::nav2_command_input());
  auto status = peer->create_publisher<ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority());
  std::size_t nonzero_count = 0;
  std::size_t zero_count = 0;
  auto output = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/amr/control/cmd_vel", amr_interfaces::qos::command(),
      [&](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        if (message->twist.linear.x == 0.0 && message->twist.angular.z == 0.0) {
          ++zero_count;
        } else {
          ++nonzero_count;
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arbitration->get_node_base_interface());
  executor.add_node(peer);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.25;
  ManipulatorStatus proof;
  proof.source_boot_id = 42;
  proof.sequence = 1;
  proof.valid = true;
  proof.state = ManipulatorStatus::STOWED_EMPTY;
  proof.base_motion_allowed = true;

  // A fresh source alone is insufficient when the factory interlock is on.
  EXPECT_FALSE(spin_until(
      executor,
      [&]() {
        input->publish(command);
        return nonzero_count > 0;
      },
      150ms));

  const auto restore_empty_stow = [&]() {
      const auto previous_nonzero = nonzero_count;
      return spin_until(
        executor,
        [&]() {
          ++proof.sequence;
          proof.valid = true;
          proof.state = ManipulatorStatus::STOWED_EMPTY;
          proof.base_motion_allowed = true;
          proof.product_attached = false;
          proof.product_id.clear();
          status->publish(proof);
          input->publish(command);
          return nonzero_count > previous_nonzero;
        },
        500ms);
    };
  const auto expect_status_stops_output = [&](ManipulatorStatus invalid) {
      const auto previous_zero = zero_count;
      status->publish(invalid);
      return spin_until(
        executor,
        [&]() {
          input->publish(command);
          return zero_count > previous_zero;
        },
        500ms);
    };

  ASSERT_TRUE(restore_empty_stow());

  for (const uint8_t state : {
      ManipulatorStatus::MOVING,
      ManipulatorStatus::DEPLOYED,
      ManipulatorStatus::FAULT})
  {
    auto invalid = proof;
    invalid.sequence = ++proof.sequence;
    invalid.state = state;
    invalid.base_motion_allowed = false;
    EXPECT_TRUE(expect_status_stops_output(invalid));
    ASSERT_TRUE(restore_empty_stow());
  }

  auto malformed = proof;
  malformed.source_boot_id = 0;
  malformed.sequence = 0;
  EXPECT_TRUE(expect_status_stops_output(malformed));
  ASSERT_TRUE(restore_empty_stow());

  auto inconsistent = proof;
  inconsistent.sequence = ++proof.sequence;
  inconsistent.state = ManipulatorStatus::STOWED_LOADED;
  inconsistent.product_attached = false;
  inconsistent.product_id.clear();
  EXPECT_TRUE(expect_status_stops_output(inconsistent));
  ASSERT_TRUE(restore_empty_stow());

  // A replay within the same boot invalidates the current proof.
  auto replay = proof;
  EXPECT_TRUE(expect_status_stops_output(replay));
  ASSERT_TRUE(restore_empty_stow());

  // A semantically consistent loaded stow also permits the existing route.
  const auto previous_nonzero = nonzero_count;
  ++proof.sequence;
  proof.state = ManipulatorStatus::STOWED_LOADED;
  proof.product_attached = true;
  proof.product_id = "product_c";
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        ++proof.sequence;
        status->publish(proof);
        input->publish(command);
        return nonzero_count > previous_nonzero;
      },
      500ms));

  // Stop refreshing only the manipulator proof; receive-time expiry must stop
  // an otherwise fresh source command.
  const auto previous_zero = zero_count;
  EXPECT_TRUE(spin_until(
      executor,
      [&]() {
        input->publish(command);
        return zero_count > previous_zero;
      },
      500ms));

  executor.remove_node(peer);
  executor.remove_node(arbitration->get_node_base_interface());
}

TEST(ControlBehavior, DockEgressSupportsSequentialReverseLegsAndClearsNav2Samples) {
  using BackUp = nav2_msgs::action::BackUp;
  using ManipulatorStatus = amr_interfaces::msg::ManipulatorStatus;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("output_frequency", 50.0),
      rclcpp::Parameter("max_linear_acceleration", 100.0),
      rclcpp::Parameter("max_angular_acceleration", 100.0),
      rclcpp::Parameter("egress_status_timeout_ms", 500),
      rclcpp::Parameter("egress_odometry_timeout_ms", 500),
      rclcpp::Parameter("egress_scan_timeout_ms", 500),
  });
  auto arbitration = std::make_shared<amr_control::CommandArbitrationNode>(options);
  ASSERT_EQ(
    arbitration->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    arbitration->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto peer = std::make_shared<rclcpp::Node>("dock_egress_behavior_peer");
  auto status = peer->create_publisher<ManipulatorStatus>(
    "/amr/manipulation/status", amr_interfaces::qos::authority());
  auto base = peer->create_publisher<amr_interfaces::msg::BaseStatus>(
    "/amr/base/status", amr_interfaces::qos::diagnostic());
  auto odom = peer->create_publisher<nav_msgs::msg::Odometry>(
    "/amr/localization/odometry", amr_interfaces::qos::sensor());
  auto scan = peer->create_publisher<sensor_msgs::msg::LaserScan>(
    "/amr/sensors/rear_lidar/scan", amr_interfaces::qos::sensor());
  auto nav2 = peer->create_publisher<geometry_msgs::msg::Twist>(
    "/amr/mpc/cmd_vel", amr_interfaces::qos::nav2_command_input());
  auto client = rclcpp_action::create_client<BackUp>(
    peer, "/amr/control/dock_egress");
  std::vector<geometry_msgs::msg::TwistStamped> outputs;
  auto output = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/amr/control/cmd_vel", amr_interfaces::qos::command(),
    [&](geometry_msgs::msg::TwistStamped::SharedPtr message) {
      outputs.push_back(*message);
    });

  ManipulatorStatus proof;
  proof.source_boot_id = 77;
  proof.valid = true;
  proof.state = ManipulatorStatus::STOWED_LOADED;
  proof.base_motion_allowed = true;
  proof.product_attached = true;
  proof.product_id = "product_a";
  amr_interfaces::msg::BaseStatus base_proof;
  base_proof.source_boot_id = 88;
  base_proof.valid = true;
  base_proof.state = base_proof.READY;
  base_proof.reason = base_proof.REASON_READY;
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.orientation.w = 1.0;
  sensor_msgs::msg::LaserScan rear_scan;
  rear_scan.header.frame_id = "base_footprint";
  rear_scan.angle_min = -2.4F;
  rear_scan.angle_max = 2.4F;
  rear_scan.angle_increment = (rear_scan.angle_max - rear_scan.angle_min) / 719.0F;
  rear_scan.range_min = 0.2F;
  rear_scan.range_max = 20.0F;
  rear_scan.ranges.assign(720, 10.0F);
  rear_scan.ranges[360] = 0.20F;
  std::atomic<bool> move{false};
  auto evidence_timer = peer->create_wall_timer(20ms, [&]() {
      ++proof.sequence;
      status->publish(proof);
      ++base_proof.sequence;
      base->publish(base_proof);
      if (move.load()) {
        odometry.pose.pose.position.x -= 0.012;
      }
      odometry.header.stamp = peer->now();
      odom->publish(odometry);
      rear_scan.header.stamp = peer->now();
      scan->publish(rear_scan);
      if (move.load()) {
        geometry_msgs::msg::Twist stale;
        stale.linear.x = 0.30;
        nav2->publish(stale);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arbitration->get_node_base_interface());
  executor.add_node(peer);
  ASSERT_TRUE(spin_until(executor, [&]() { return proof.sequence > 5; }, 2s));
  ASSERT_TRUE(client->wait_for_action_server(1s));
  BackUp::Goal goal;
  goal.target.x = 0.50;
  goal.speed = 0.10F;
  goal.time_allowance.sec = 60;
  auto sent = client->async_send_goal(goal);
  ASSERT_TRUE(spin_until(
    executor, [&]() { return sent.wait_for(0ms) == std::future_status::ready; }, 2s));
  auto handle = sent.get();
  ASSERT_TRUE(handle);
  move.store(true);
  auto result = client->async_get_result(handle);
  ASSERT_TRUE(spin_until(
    executor, [&]() { return result.wait_for(0ms) == std::future_status::ready; }, 3s));
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wrapped.result);
  ASSERT_TRUE(spin_until(executor, [&]() { return outputs.size() > 3; }, 500ms));
  bool saw_reverse = false;
  for (const auto & command : outputs) {
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
    EXPECT_LE(command.twist.linear.x, 0.0);
    EXPECT_LE(std::abs(command.twist.linear.x), 0.10 + 1e-6);
    if (command.twist.linear.x < -1e-4) saw_reverse = true;
  }
  EXPECT_TRUE(saw_reverse);

  const auto first_leg_output_count = outputs.size();
  BackUp::Goal second_goal;
  second_goal.target.x = 0.40;
  second_goal.speed = 0.10F;
  second_goal.time_allowance.sec = 60;
  auto second_sent = client->async_send_goal(second_goal);
  ASSERT_TRUE(spin_until(
    executor, [&]() {
      return second_sent.wait_for(0ms) == std::future_status::ready;
    }, 2s));
  auto second_handle = second_sent.get();
  ASSERT_TRUE(second_handle);
  auto second_result = client->async_get_result(second_handle);
  ASSERT_TRUE(spin_until(
    executor, [&]() {
      return second_result.wait_for(0ms) == std::future_status::ready;
    }, 3s));
  const auto second_wrapped = second_result.get();
  EXPECT_EQ(second_wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(second_wrapped.result);
  ASSERT_TRUE(spin_until(
    executor, [&]() { return outputs.size() > first_leg_output_count + 3; }, 500ms));
  saw_reverse = false;
  for (std::size_t index = first_leg_output_count; index < outputs.size(); ++index) {
    const auto & command = outputs[index];
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
    EXPECT_LE(command.twist.linear.x, 0.0);
    EXPECT_LE(std::abs(command.twist.linear.x), 0.10 + 1e-6);
    if (command.twist.linear.x < -1e-4) saw_reverse = true;
  }
  EXPECT_TRUE(saw_reverse);

  executor.remove_node(peer);
  executor.remove_node(arbitration->get_node_base_interface());
}

TEST(ControlBehavior, DockEgressRejectsObstructedAndMalformedRequests) {
  using BackUp = nav2_msgs::action::BackUp;
  using ManipulatorStatus = amr_interfaces::msg::ManipulatorStatus;
  auto arbitration = std::make_shared<amr_control::CommandArbitrationNode>();
  ASSERT_EQ(
    arbitration->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    arbitration->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto peer = std::make_shared<rclcpp::Node>("dock_egress_rejection_peer");
  auto status = peer->create_publisher<ManipulatorStatus>(
    "/amr/manipulation/status", amr_interfaces::qos::authority());
  auto base = peer->create_publisher<amr_interfaces::msg::BaseStatus>(
    "/amr/base/status", amr_interfaces::qos::diagnostic());
  auto odom = peer->create_publisher<nav_msgs::msg::Odometry>(
    "/amr/localization/odometry", amr_interfaces::qos::sensor());
  auto scan = peer->create_publisher<sensor_msgs::msg::LaserScan>(
    "/amr/sensors/rear_lidar/scan", amr_interfaces::qos::sensor());
  auto client = rclcpp_action::create_client<BackUp>(peer, "/amr/control/dock_egress");
  ManipulatorStatus proof;
  proof.source_boot_id = 7; proof.valid = true;
  proof.state = ManipulatorStatus::STOWED_LOADED;
  proof.base_motion_allowed = true; proof.product_attached = true;
  proof.product_id = "product_a";
  amr_interfaces::msg::BaseStatus base_proof;
  base_proof.source_boot_id = 8; base_proof.valid = true;
  base_proof.state = base_proof.READY; base_proof.reason = base_proof.REASON_READY;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  sensor_msgs::msg::LaserScan rear_scan;
  rear_scan.header.frame_id = "base_footprint";
  rear_scan.angle_min = -2.4F; rear_scan.angle_max = 2.4F;
  rear_scan.angle_increment = (rear_scan.angle_max - rear_scan.angle_min) / 719.0F;
  rear_scan.range_min = 0.2F; rear_scan.range_max = 20.0F;
  rear_scan.ranges.assign(720, 10.0F);
  auto timer = peer->create_wall_timer(20ms, [&]() {
      ++proof.sequence; status->publish(proof);
      ++base_proof.sequence; base->publish(base_proof);
      odometry.header.stamp = peer->now(); odom->publish(odometry);
      rear_scan.header.stamp = peer->now(); scan->publish(rear_scan);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arbitration->get_node_base_interface());
  executor.add_node(peer);
  ASSERT_TRUE(spin_until(executor, [&]() { return proof.sequence > 5; }, 2s));
  ASSERT_TRUE(client->wait_for_action_server(1s));

  BackUp::Goal malformed;
  malformed.target.x = 0.50;
  malformed.target.y = 0.01;
  malformed.speed = 0.10F;
  malformed.time_allowance.sec = 60;
  auto malformed_future = client->async_send_goal(malformed);
  ASSERT_TRUE(spin_until(
    executor, [&]() { return malformed_future.wait_for(0ms) == std::future_status::ready; }, 1s));
  EXPECT_FALSE(malformed_future.get());

  rear_scan.ranges[0] = 0.20F;
  rear_scan.header.stamp = peer->now();
  scan->publish(rear_scan);
  std::this_thread::sleep_for(50ms);
  BackUp::Goal obstructed;
  obstructed.target.x = 0.50;
  obstructed.speed = 0.10F;
  obstructed.time_allowance.sec = 60;
  auto obstructed_future = client->async_send_goal(obstructed);
  ASSERT_TRUE(spin_until(
    executor, [&]() { return obstructed_future.wait_for(0ms) == std::future_status::ready; }, 1s));
  EXPECT_FALSE(obstructed_future.get());
  executor.remove_node(peer);
  executor.remove_node(arbitration->get_node_base_interface());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
