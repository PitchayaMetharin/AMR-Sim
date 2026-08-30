#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace amr_perception {

inline std::size_t field_size(uint8_t datatype) {
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

inline bool has_valid_xyz_fields(const sensor_msgs::msg::PointCloud2 & cloud) {
  std::array<const sensor_msgs::msg::PointField *, 3> xyz{};
  for (const auto & field : cloud.fields) {
    std::size_t index = xyz.size();
    if (field.name == "x") index = 0;
    else if (field.name == "y") index = 1;
    else if (field.name == "z") index = 2;
    if (index == xyz.size()) continue;
    if (xyz[index] != nullptr || field.count != 1 ||
        field_size(field.datatype) == 0 ||
        static_cast<std::size_t>(field.offset) +
        field_size(field.datatype) > cloud.point_step) {
      return false;
    }
    xyz[index] = &field;
  }
  if (xyz[0] == nullptr || xyz[1] == nullptr || xyz[2] == nullptr) {
    return false;
  }
  for (std::size_t first = 0; first < xyz.size(); ++first) {
    const auto first_begin = xyz[first]->offset;
    const auto first_end = first_begin + field_size(xyz[first]->datatype);
    for (std::size_t second = first + 1; second < xyz.size(); ++second) {
      const auto second_begin = xyz[second]->offset;
      const auto second_end =
        second_begin + field_size(xyz[second]->datatype);
      if (first_begin < second_end && second_begin < first_end) {
        return false;
      }
    }
  }
  return true;
}

inline bool has_valid_layout(const sensor_msgs::msg::PointCloud2 & cloud) {
  if (cloud.header.frame_id.empty() ||
      (cloud.header.stamp.sec == 0 && cloud.header.stamp.nanosec == 0) ||
      cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0 ||
      static_cast<std::size_t>(cloud.row_step) <
      static_cast<std::size_t>(cloud.width) * cloud.point_step ||
      !has_valid_xyz_fields(cloud)) {
    return false;
  }
  return cloud.data.size() ==
         static_cast<std::size_t>(cloud.row_step) * cloud.height;
}

}  // namespace amr_perception
