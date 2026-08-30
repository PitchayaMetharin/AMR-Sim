#include <cstdint>

#include <gtest/gtest.h>

#include "amr_interfaces/qos_profiles.hpp"

namespace {

uint64_t nanoseconds(const rmw_time_t & duration) {
  return duration.sec * 1000000000ULL + duration.nsec;
}

}  // namespace

TEST(QosProfiles, SensorIsBestEffortDepthFive) {
  const auto profile = amr_interfaces::qos::sensor().get_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 5U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfiles, AuthorityHasDeclaredDeadline) {
  const auto profile = amr_interfaces::qos::authority().get_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 1U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(nanoseconds(profile.deadline), 100000000ULL);
}

TEST(QosProfiles, CommandHasDeclaredDeadlineAndLifespan) {
  const auto profile = amr_interfaces::qos::command().get_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 1U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(nanoseconds(profile.deadline), 100000000ULL);
  EXPECT_EQ(nanoseconds(profile.lifespan), 200000000ULL);
}

TEST(QosProfiles, Nav2InputMatchesTheExternalPublisher) {
  const auto profile =
      amr_interfaces::qos::nav2_command_input().get_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 1U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(profile.deadline.sec, 0);
  EXPECT_EQ(profile.deadline.nsec, 0U);
  EXPECT_EQ(profile.lifespan.sec, 0);
  EXPECT_EQ(profile.lifespan.nsec, 0U);
}

TEST(QosProfiles, DiagnosticIsReliableDepthTwenty) {
  const auto profile = amr_interfaces::qos::diagnostic().get_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 20U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
}
