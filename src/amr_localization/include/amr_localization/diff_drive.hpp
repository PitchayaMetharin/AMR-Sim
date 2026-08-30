#pragma once

#include <cmath>

namespace amr_localization {

// Incremental planar displacement produced by two differential-drive wheels.
struct PlanarIncrement {
  double x;
  double y;
  double yaw;
};

inline PlanarIncrement integrate_diff_drive(
    const double yaw, const double left_delta, const double right_delta,
    const double wheel_radius, const double wheel_separation) {
  // Convert encoder angle changes to average chassis travel distance.
  const double distance =
      wheel_radius * (left_delta + right_delta) / 2.0;
  // Wheel-distance difference rotates the chassis about its center.
  const double yaw_delta =
      wheel_radius * (right_delta - left_delta) / wheel_separation;
  // Translate using the midpoint heading for this finite integration step.
  const double midpoint_yaw = yaw + yaw_delta / 2.0;
  return {
      distance * std::cos(midpoint_yaw),
      distance * std::sin(midpoint_yaw),
      yaw_delta,
  };
}

}  // namespace amr_localization
