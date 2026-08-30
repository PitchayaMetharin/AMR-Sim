#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#define main wheel_odometry_main
#include "../src/wheel_odometry_node.cpp"
#undef main

TEST(WheelOdometryConfiguration, RejectsInvalidGeometry) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter(
          "wheel_radius", std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("wheel_separation", 0.0),
  });
  auto node =
      std::make_shared<amr_localization::WheelOdometryNode>(options);
  const auto state = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
      state.id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
