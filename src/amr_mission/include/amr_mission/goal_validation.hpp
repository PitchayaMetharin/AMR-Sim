#pragma once

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"

namespace amr_mission {

inline bool valid_planar_pose(const geometry_msgs::msg::Pose & pose) {
  const bool finite =
      std::isfinite(pose.position.x) &&
      std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) &&
      std::isfinite(pose.orientation.x) &&
      std::isfinite(pose.orientation.y) &&
      std::isfinite(pose.orientation.z) &&
      std::isfinite(pose.orientation.w);
  if (!finite || pose.position.z != 0.0 ||
      pose.orientation.x != 0.0 || pose.orientation.y != 0.0) {
    return false;
  }
  const double norm_squared =
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
  return std::abs(norm_squared - 1.0) <= 1e-6;
}

}  // namespace amr_mission
