#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_perception {

// Sensor pose in the merged output frame (planar).
struct SensorPose2D {
  double x;
  double y;
  double yaw;
};

struct ScanSource {
  sensor_msgs::msg::LaserScan scan;
  SensorPose2D pose;
};

struct MergedScanLayout {
  double angle_min;
  double angle_increment;
  std::size_t bin_count;
  double range_min;
  double range_max;
};

namespace detail {

constexpr double kTwoPi = 6.283185307179586476925286766559;

inline bool source_is_valid(const ScanSource & source) {
  const auto & scan = source.scan;
  const double increment = scan.angle_increment;
  if (!std::isfinite(increment) || increment <= 0.0) return false;
  if (!std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max)) return false;
  if (!std::isfinite(source.pose.x) || !std::isfinite(source.pose.y) ||
      !std::isfinite(source.pose.yaw)) {
    return false;
  }
  const double expected =
    std::round((static_cast<double>(scan.angle_max) - scan.angle_min) / increment) + 1.0;
  return expected >= 1.0 && expected == static_cast<double>(scan.ranges.size());
}

}  // namespace detail

// Merge planar scans into one 360 degree scan in the output frame. Returns
// false and leaves `out` untouched if any input is invalid. out.header is
// never modified.
inline bool merge_scans(
  const std::vector<ScanSource> & sources, const MergedScanLayout & layout,
  sensor_msgs::msg::LaserScan & out) {
  if (sources.empty() || layout.bin_count == 0) return false;
  if (!std::isfinite(layout.angle_increment) || layout.angle_increment <= 0.0) return false;
  if (!std::isfinite(layout.angle_min) || !std::isfinite(layout.range_min) ||
      !std::isfinite(layout.range_max) || layout.range_max <= layout.range_min) {
    return false;
  }
  for (const auto & source : sources) {
    if (!detail::source_is_valid(source)) return false;
  }

  std::vector<float> ranges(
    layout.bin_count, std::numeric_limits<float>::infinity());
  for (const auto & source : sources) {
    const auto & scan = source.scan;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double angle = source.pose.yaw + scan.angle_min +
        static_cast<double>(i) * scan.angle_increment;
      const double x = source.pose.x + range * std::cos(angle);
      const double y = source.pose.y + range * std::sin(angle);
      const double distance = std::hypot(x, y);
      if (!std::isfinite(distance) || distance < layout.range_min ||
          distance > layout.range_max) {
        continue;
      }
      double offset = std::fmod(std::atan2(y, x) - layout.angle_min, detail::kTwoPi);
      if (offset < 0.0) offset += detail::kTwoPi;
      const std::size_t bin =
        static_cast<std::size_t>(std::floor(offset / layout.angle_increment)) %
        layout.bin_count;
      ranges[bin] = std::min(ranges[bin], static_cast<float>(distance));
    }
  }

  out.angle_min = static_cast<float>(layout.angle_min);
  out.angle_increment = static_cast<float>(layout.angle_increment);
  out.angle_max = static_cast<float>(
    layout.angle_min + static_cast<double>(layout.bin_count - 1) * layout.angle_increment);
  out.range_min = static_cast<float>(layout.range_min);
  out.range_max = static_cast<float>(layout.range_max);
  out.time_increment = 0.0f;
  out.scan_time = 0.0f;
  out.ranges = std::move(ranges);
  out.intensities.clear();
  return true;
}

// True only for a non-empty, fresh, non-future set of stamps whose spread is
// within max_skew_sec.
inline bool stamps_usable(
  const std::vector<double> & stamps_sec, double now_sec, double max_age_sec,
  double max_skew_sec) {
  if (stamps_sec.empty() || !std::isfinite(now_sec) || !std::isfinite(max_age_sec) ||
      !std::isfinite(max_skew_sec) || max_age_sec <= 0.0 || max_skew_sec < 0.0) {
    return false;
  }
  double lowest = std::numeric_limits<double>::infinity();
  double highest = -std::numeric_limits<double>::infinity();
  for (const double stamp : stamps_sec) {
    if (!std::isfinite(stamp) || stamp <= 0.0 || stamp > now_sec ||
        now_sec - stamp > max_age_sec) {
      return false;
    }
    lowest = std::min(lowest, stamp);
    highest = std::max(highest, stamp);
  }
  return highest - lowest <= max_skew_sec;
}

}  // namespace amr_perception
