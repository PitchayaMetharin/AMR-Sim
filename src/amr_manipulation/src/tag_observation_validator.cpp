#include "amr_manipulation/tag_observation_validator.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace amr_manipulation {
namespace {

constexpr std::size_t kRequiredObservations = 5U;
constexpr double kMaximumAgeSeconds = 0.250;
constexpr double kMaximumWindowSeconds = 1.0;
constexpr double kMaximumPositionSpreadMeters = 0.015;
constexpr double kMaximumOrientationSpreadRadians = 0.05;

const std::set<std::int32_t> kConfiguredIds{10, 11, 12, 20, 101, 102, 103};

double position_distance(const TagObservation & left, const TagObservation & right) {
  double squared = 0.0;
  for (std::size_t index = 0; index < left.position.size(); ++index) {
    const double delta = left.position[index] - right.position[index];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

double orientation_distance(const TagObservation & left, const TagObservation & right) {
  double left_norm_squared = 0.0;
  double right_norm_squared = 0.0;
  double dot = 0.0;
  for (std::size_t index = 0; index < left.orientation_xyzw.size(); ++index) {
    left_norm_squared += left.orientation_xyzw[index] * left.orientation_xyzw[index];
    right_norm_squared += right.orientation_xyzw[index] * right.orientation_xyzw[index];
    dot += left.orientation_xyzw[index] * right.orientation_xyzw[index];
  }
  if (left_norm_squared <= 0.0 || right_norm_squared <= 0.0) {
    return INFINITY;
  }
  dot /= std::sqrt(left_norm_squared * right_norm_squared);
  return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

TagValidationResult reject(const std::string & detail) {
  return {false, detail};
}

}  // namespace

TagValidationResult validate_tag_observations(
  const std::int32_t expected_id,
  const std::vector<TagObservation> & observations)
{
  if (kConfiguredIds.count(expected_id) == 0U) {
    return reject("expected tag is not in the station/product registry");
  }
  if (observations.size() < kRequiredObservations) {
    return reject("fewer than five observations");
  }

  const auto first = observations.end() - static_cast<std::ptrdiff_t>(kRequiredObservations);
  const std::vector<TagObservation> window(first, observations.end());
  for (const auto & observation : window) {
    if (observation.family != "tag36h11" || observation.id != expected_id) {
      return reject("wrong tag family or ID");
    }
    if (observation.hamming != 0) {
      return reject("tag required bit correction");
    }
    if (!std::isfinite(observation.receive_age_seconds) ||
      observation.receive_age_seconds < 0.0 ||
      observation.receive_age_seconds > kMaximumAgeSeconds)
    {
      return reject("tag observation is stale or has invalid age");
    }
  }

  const auto [minimum_time, maximum_time] = std::minmax_element(
    window.begin(), window.end(),
    [](const TagObservation & left, const TagObservation & right) {
      return left.steady_receive_seconds < right.steady_receive_seconds;
    });
  if (!std::isfinite(minimum_time->steady_receive_seconds) ||
    !std::isfinite(maximum_time->steady_receive_seconds) ||
    maximum_time->steady_receive_seconds - minimum_time->steady_receive_seconds >
    kMaximumWindowSeconds)
  {
    return reject("five observations were not collected within one second");
  }

  for (auto left = window.begin(); left != window.end(); ++left) {
    for (auto right = std::next(left); right != window.end(); ++right) {
      if (position_distance(*left, *right) > kMaximumPositionSpreadMeters) {
        return reject("tag position spread exceeds 15 mm");
      }
      if (orientation_distance(*left, *right) > kMaximumOrientationSpreadRadians) {
        return reject("tag orientation spread exceeds 0.05 rad");
      }
    }
  }
  return {true, "five fresh stable zero-hamming observations"};
}

}  // namespace amr_manipulation
