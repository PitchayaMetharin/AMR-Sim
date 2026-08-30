#include <cmath>
#include <cstdint>
#include <vector>

#include "amr_manipulation/tag_observation_validator.hpp"
#include "gtest/gtest.h"

namespace {

using amr_manipulation::TagObservation;

std::vector<TagObservation> stable_observations(const std::int32_t id) {
  std::vector<TagObservation> observations;
  for (int index = 0; index < 5; ++index) {
    TagObservation observation;
    observation.id = id;
    observation.receive_age_seconds = 0.02;
    observation.steady_receive_seconds = 10.0 + 0.1 * index;
    observation.position = {1.0 + 0.001 * index, 2.0, 0.5};
    observations.push_back(observation);
  }
  return observations;
}

TEST(TagObservationValidator, AcceptsEveryConfiguredStationAndProductTag) {
  for (const std::int32_t id : {10, 11, 12, 20, 101, 102, 103}) {
    EXPECT_TRUE(amr_manipulation::validate_tag_observations(
      id, stable_observations(id)).accepted) << id;
  }
}

TEST(TagObservationValidator, RejectsUnknownWrongStaleCorrectedAndIncompleteTags) {
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(
    999, stable_observations(999)).accepted);
  auto wrong = stable_observations(101);
  wrong.back().id = 102;
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, wrong).accepted);
  auto stale = stable_observations(101);
  stale.back().receive_age_seconds = 0.251;
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, stale).accepted);
  auto corrected = stable_observations(101);
  corrected.back().hamming = 1;
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, corrected).accepted);
  auto incomplete = stable_observations(101);
  incomplete.pop_back();
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, incomplete).accepted);
}

TEST(TagObservationValidator, RejectsSlowUnstableOrMalformedObservationWindows) {
  auto slow = stable_observations(101);
  slow.back().steady_receive_seconds = 11.01;
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, slow).accepted);
  auto position_spread = stable_observations(101);
  position_spread.back().position[0] += 0.020;
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, position_spread).accepted);
  auto orientation_spread = stable_observations(101);
  orientation_spread.back().orientation_xyzw = {0.0, 0.0, std::sin(0.03), std::cos(0.03)};
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, orientation_spread).accepted);
  auto zero_quaternion = stable_observations(101);
  zero_quaternion.back().orientation_xyzw = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(amr_manipulation::validate_tag_observations(101, zero_quaternion).accepted);
}

}  // namespace
