#include <chrono>
#include <limits>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "amr_interfaces/qos_profiles.hpp"

#define main health_supervisor_main
#include "../src/health_supervisor_node.cpp"
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

TEST(HealthConfiguration, RejectsInvalidNumericParameters) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("evidence_timeout_ms", 0),
      rclcpp::Parameter(
          "output_frequency", std::numeric_limits<double>::infinity()),
  });
  auto node =
      std::make_shared<amr_health::HealthSupervisorNode>(options);
  const auto state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
      state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(HealthBehavior, EnforcesBootSequenceAndTimeEvidenceRules) {
  using BaseStatus = amr_interfaces::msg::BaseStatus;
  using HealthStatus = amr_interfaces::msg::HealthStatus;

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("evidence_timeout_ms", 100),
      rclcpp::Parameter("output_frequency", 50.0),
  });
  auto supervisor =
      std::make_shared<amr_health::HealthSupervisorNode>(options);
  ASSERT_EQ(
      supervisor->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
      supervisor->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto peer = std::make_shared<rclcpp::Node>("health_behavior_peer");
  auto base_pub = peer->create_publisher<BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic());
  uint8_t health_state = HealthStatus::UNKNOWN;
  uint16_t health_reason = HealthStatus::REASON_UNAVAILABLE;
  bool received_health = false;
  auto health_sub = peer->create_subscription<HealthStatus>(
      "/amr/health/status", amr_interfaces::qos::diagnostic(),
      [&](HealthStatus::SharedPtr message) {
        health_state = message->state;
        health_reason = message->reason;
        received_health = true;
      });

  BaseStatus base;
  base.source_boot_id = 11;
  base.valid = true;
  base.state = BaseStatus::READY;
  base.reason = BaseStatus::REASON_READY;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor->get_node_base_interface());
  executor.add_node(peer);
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        return received_health &&
          health_reason == HealthStatus::REASON_EVIDENCE_MISSING_OR_STALE;
      },
      2s));

  auto publish_fresh = [&]() {
      const auto stamp = peer->now();
      base.header.stamp = stamp;
      base.sequence++;
      base_pub->publish(base);
    };
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        publish_fresh();
        return health_state == HealthStatus::HEALTHY;
      },
      2s));

  const auto accepted_base_sequence = base.sequence;
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        const auto stamp = peer->now();
        base.header.stamp = stamp;
        base.sequence = accepted_base_sequence;
        base_pub->publish(base);
        return health_reason == HealthStatus::REASON_INVALID_EVIDENCE;
      },
      500ms));

  base.sequence = accepted_base_sequence;
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        publish_fresh();
        return health_state == HealthStatus::HEALTHY;
      },
      500ms));
  const auto accepted_stamp = base.header.stamp;
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        base.sequence++;
        base.header.stamp = accepted_stamp;
        if (base.header.stamp.nanosec > 0) {
          base.header.stamp.nanosec--;
        } else {
          base.header.stamp.sec--;
          base.header.stamp.nanosec = 999999999;
        }
        base_pub->publish(base);
        return health_reason == HealthStatus::REASON_BACKWARD_TIME;
      },
      500ms));

  base.source_boot_id++;
  base.sequence = 0;
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        publish_fresh();
        return health_state == HealthStatus::HEALTHY;
      },
      500ms));

  executor.remove_node(peer);
  executor.remove_node(supervisor->get_node_base_interface());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
