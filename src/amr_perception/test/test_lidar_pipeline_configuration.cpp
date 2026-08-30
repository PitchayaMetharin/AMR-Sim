#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#define AMR_SENSOR_ID "configuration_test"
#define main lidar_pipeline_main
#include "../src/lidar_pipeline_node.cpp"
#undef main

TEST(LidarPipelineConfiguration, RejectsNonfiniteMaximumAge) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter(
          "max_age_seconds", std::numeric_limits<double>::quiet_NaN()),
  });
  auto node =
      std::make_shared<amr_perception::LidarPipelineNode>(options);
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
