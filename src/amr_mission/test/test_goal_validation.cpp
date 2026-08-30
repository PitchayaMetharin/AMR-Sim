#include <limits>

#include <gtest/gtest.h>

#include "amr_mission/goal_validation.hpp"

namespace {

geometry_msgs::msg::Pose valid_pose() {
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = -2.0;
  pose.orientation.z = 0.7071067811865476;
  pose.orientation.w = 0.7071067811865476;
  return pose;
}

}  // namespace

TEST(GoalValidation, AcceptsFiniteNormalizedPlanarPose) {
  EXPECT_TRUE(amr_mission::valid_planar_pose(valid_pose()));
}

TEST(GoalValidation, RejectsEveryNonfiniteComponent) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int component = 0; component < 7; ++component) {
    auto pose = valid_pose();
    double * values[] = {
      &pose.position.x, &pose.position.y, &pose.position.z,
      &pose.orientation.x, &pose.orientation.y,
      &pose.orientation.z, &pose.orientation.w};
    *values[component] = nan;
    EXPECT_FALSE(amr_mission::valid_planar_pose(pose)) << component;
  }
}

TEST(GoalValidation, RejectsNonplanarOrUnnormalizedPose) {
  auto pose = valid_pose();
  pose.position.z = 0.1;
  EXPECT_FALSE(amr_mission::valid_planar_pose(pose));

  pose = valid_pose();
  pose.orientation.x = 0.1;
  EXPECT_FALSE(amr_mission::valid_planar_pose(pose));

  pose = valid_pose();
  pose.orientation.w = 0.5;
  EXPECT_FALSE(amr_mission::valid_planar_pose(pose));
}
