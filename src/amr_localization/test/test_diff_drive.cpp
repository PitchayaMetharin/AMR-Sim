#include <cmath>

#include "amr_localization/diff_drive.hpp"
#include "gtest/gtest.h"

namespace {
constexpr double kWheelRadius = 0.1128;
constexpr double kWheelSeparation = 0.566;
}

TEST(DiffDrive, StraightTravel) {
  const auto result =
      amr_localization::integrate_diff_drive(
          0.0, 2.0, 2.0, kWheelRadius, kWheelSeparation);
  EXPECT_NEAR(result.x, 2.0 * kWheelRadius, 1e-12);
  EXPECT_NEAR(result.y, 0.0, 1e-12);
  EXPECT_NEAR(result.yaw, 0.0, 1e-12);
}

TEST(DiffDrive, InPlaceTurn) {
  const auto result =
      amr_localization::integrate_diff_drive(
          0.0, -1.0, 1.0, kWheelRadius, kWheelSeparation);
  EXPECT_NEAR(result.x, 0.0, 1e-12);
  EXPECT_NEAR(result.y, 0.0, 1e-12);
  EXPECT_NEAR(result.yaw, 2.0 * kWheelRadius / kWheelSeparation, 1e-12);
}

TEST(DiffDrive, ArcUsesMidpointHeading) {
  const auto result =
      amr_localization::integrate_diff_drive(
          0.2, 1.0, 2.0, kWheelRadius, kWheelSeparation);
  const double yaw_delta = kWheelRadius / kWheelSeparation;
  const double distance = 1.5 * kWheelRadius;
  EXPECT_NEAR(result.x, distance * std::cos(0.2 + yaw_delta / 2.0), 1e-12);
  EXPECT_NEAR(result.y, distance * std::sin(0.2 + yaw_delta / 2.0), 1e-12);
  EXPECT_NEAR(result.yaw, yaw_delta, 1e-12);
}
