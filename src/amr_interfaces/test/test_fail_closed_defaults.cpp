#include <gtest/gtest.h>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/health_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"

TEST(FailClosedDefaults, StatusMessagesAreInvalidAndUnknown)
{
  const amr_interfaces::msg::BaseStatus base;
  EXPECT_EQ(base.sequence, 0U);
  EXPECT_FALSE(base.valid);
  EXPECT_EQ(base.source_boot_id, 0U);
  EXPECT_EQ(base.state, amr_interfaces::msg::BaseStatus::UNKNOWN);
  EXPECT_EQ(base.reason, amr_interfaces::msg::BaseStatus::REASON_UNAVAILABLE);

  const amr_interfaces::msg::HealthStatus health;
  EXPECT_EQ(health.sequence, 0U);
  EXPECT_FALSE(health.valid);
  EXPECT_EQ(health.source_boot_id, 0U);
  EXPECT_EQ(health.state, amr_interfaces::msg::HealthStatus::UNKNOWN);
  EXPECT_EQ(
    health.reason,
    amr_interfaces::msg::HealthStatus::REASON_UNAVAILABLE);
  EXPECT_FALSE(health.base_ready);

  const amr_interfaces::msg::ManipulatorStatus manipulator;
  EXPECT_EQ(manipulator.source_boot_id, 0U);
  EXPECT_EQ(manipulator.sequence, 0U);
  EXPECT_FALSE(manipulator.valid);
  EXPECT_EQ(
    manipulator.state,
    amr_interfaces::msg::ManipulatorStatus::STARTING);
  EXPECT_FALSE(manipulator.base_motion_allowed);
  EXPECT_FALSE(manipulator.product_attached);
  EXPECT_TRUE(manipulator.product_id.empty());
}
