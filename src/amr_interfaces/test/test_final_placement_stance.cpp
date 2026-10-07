#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "amr_interfaces/final_placement_stance.hpp"

namespace {

// Independent characterization of the pre-extraction gate6 calculation.
std::array<double, 3> original_stance(
  int id, const std::array<double, 3> & dock, const std::array<double, 3> & slot)
{
  const double lateral = slot[1] - dock[1];
  const bool centered_102 = id == 102 && lateral == 0.0;
  double x;
  double y;
  if (centered_102) {
    x = 0.748000000;
    y = 0.100000000;
  } else if (lateral > 0.0) {
    x = 0.520000000;
    y = -0.640000000;
  } else if (lateral < 0.0) {
    x = 0.520000000;
    y = 0.580000000;
  } else {
    x = 1.0;
    y = 0.0;
  }
  const double direction_radius = std::hypot(x, y);
  const double radius = centered_102 ? direction_radius : 0.785 - 0.07 - 0.005;
  const double scale = radius / direction_radius;
  const double base_x = x * scale;
  const double base_y = y * scale;
  const double map_x = std::cos(dock[2]) * base_x - std::sin(dock[2]) * base_y;
  const double map_y = std::sin(dock[2]) * base_x + std::cos(dock[2]) * base_y;
  return {slot[0] - map_x, slot[1] - map_y, dock[2]};
}

using amr_interfaces::placement::final_placement_stance;

}  // namespace

TEST(FinalPlacementStance, RegisteredABNumericOracles)
{
  const std::array<double, 3> dock{-3.4, 0.0, std::acos(-1.0)};
  const auto a = final_placement_stance(101, dock, {-4.10, 0.50, 0.075});
  EXPECT_NEAR(a.physical[0], -3.652279236182929, 1e-14);
  EXPECT_NEAR(a.physical[1], -0.05104094008254856, 1e-14);
  EXPECT_DOUBLE_EQ(a.physical[2], dock[2]);
  EXPECT_FALSE(a.product102_center_slot);
  const auto b = final_placement_stance(102, dock, {-4.10, 0.0, 0.075});
  EXPECT_NEAR(b.physical[0], -3.352, 1e-14);
  EXPECT_NEAR(b.physical[1], 0.100, 1e-14);
  EXPECT_DOUBLE_EQ(b.physical[2], dock[2]);
  EXPECT_TRUE(b.product102_center_slot);
  EXPECT_NEAR(b.base_radius, 0.754654888, 1e-9);
}

TEST(FinalPlacementStance, AllProductsSlotsAndHeadingsMatchOriginalMath)
{
  for (int id : {101, 102, 103}) {
    for (double yaw : {0.0, 0.73, -1.2, std::acos(-1.0)}) {
      const std::array<double, 3> dock{-3.4, 0.125, yaw};
      for (double offset : {-0.50, 0.0, 0.50}) {
        SCOPED_TRACE(::testing::Message() << "product=" << id << " yaw=" << yaw
                     << " lateral=" << offset);
        const std::array<double, 3> slot{-4.10, dock[1] + offset, 0.075};
        const auto expected = original_stance(id, dock, slot);
        const auto actual = final_placement_stance(id, dock, slot);
        for (std::size_t i = 0; i < 3; ++i) {
          EXPECT_DOUBLE_EQ(actual.physical[i], expected[i]);
        }
        EXPECT_EQ(actual.product102_center_slot, id == 102 && offset == 0.0);
        EXPECT_DOUBLE_EQ(actual.lateral_offset, offset);
        EXPECT_NEAR(std::hypot(actual.base_x, actual.base_y), actual.base_radius, 1e-15);
        EXPECT_LE(actual.base_radius, 0.785);
      }
    }
  }
}

TEST(FinalPlacementStance, RegisteredLowerAndGenericCenterNumericOracles)
{
  const std::array<double, 3> dock{-3.4, 0.0, std::acos(-1.0)};
  const auto lower = final_placement_stance(103, dock, {-4.10, -0.50, 0.075});
  EXPECT_NEAR(lower.physical[0], -3.626043038816061, 1e-14);
  EXPECT_NEAR(lower.physical[1], 0.02864430285900821, 1e-14);
  for (int id : {101, 103}) {
    const auto center = final_placement_stance(id, dock, {-4.10, 0.0, 0.075});
    EXPECT_NEAR(center.physical[0], -3.390, 1e-14);
    EXPECT_NEAR(center.physical[1], 0.0, 1e-14);
    EXPECT_FALSE(center.product102_center_slot);
  }
}

TEST(FinalPlacementStance, RejectsUnsupportedProduct)
{
  for (int id : {0, 100, 104, -101}) {
    EXPECT_THROW(final_placement_stance(id, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.075}),
      std::runtime_error);
  }
}

TEST(FinalPlacementStance, RejectsEveryNonfiniteInputCoordinate)
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int id : {101, 102, 103}) {
    for (double invalid : {inf, -inf, nan}) {
      for (std::size_t i = 0; i < 3; ++i) {
        std::array<double, 3> dock{-3.4, 0.0, std::acos(-1.0)};
        std::array<double, 3> slot{-4.10, 0.50, 0.075};
        dock[i] = invalid;
        EXPECT_THROW(final_placement_stance(id, dock, slot), std::runtime_error);
        dock = {-3.4, 0.0, std::acos(-1.0)};
        slot[i] = invalid;
        EXPECT_THROW(final_placement_stance(id, dock, slot), std::runtime_error);
      }
    }
  }
}

TEST(FinalPlacementStance, RejectsOverflowingLateralOffset)
{
  const double max = std::numeric_limits<double>::max();
  EXPECT_THROW(final_placement_stance(101, {0.0, -max, 0.0}, {1.0, max, 0.075}),
    std::runtime_error);
}
