#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "amr_interfaces/qos_profiles.hpp"

#define main base_adapter_main
#include "../src/base_adapter_node.cpp"
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

TEST(BaseAdapterConfiguration, RejectsNonpositiveTimeouts) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("gated_command_timeout_ms", 0),
      rclcpp::Parameter("input_timeout_ms", -1),
  });
  auto node =
      std::make_shared<amr_base_adapter::BaseAdapterNode>(options);
  const auto state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
      state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(BaseAdapterBehavior, ExpiresTheLastControlCommandToZero) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("gated_command_timeout_ms", 80),
      rclcpp::Parameter("input_timeout_ms", 300),
  });
  auto adapter =
      std::make_shared<amr_base_adapter::BaseAdapterNode>(options);
  ASSERT_EQ(
      adapter->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
      adapter->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto peer = std::make_shared<rclcpp::Node>("base_adapter_behavior_peer");
  auto input = peer->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/amr/control/cmd_vel", amr_interfaces::qos::command());
  bool saw_nonzero = false;
  bool saw_zero_after_nonzero = false;
  auto output = peer->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/amr/simulation/base/cmd_vel", amr_interfaces::qos::command(),
      [&](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        if (message->twist.linear.x != 0.0) {
          saw_nonzero = true;
        } else if (saw_nonzero) {
          saw_zero_after_nonzero = true;
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter->get_node_base_interface());
  executor.add_node(peer);
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = "base_footprint";
  command.twist.linear.x = 0.25;
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
  executor.remove_node(adapter->get_node_base_interface());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
