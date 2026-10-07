#ifndef AMR_INTERFACES__FINAL_PLACEMENT_STANCE_HPP_
#define AMR_INTERFACES__FINAL_PLACEMENT_STANCE_HPP_

#include <array>
#include <cmath>
#include <stdexcept>

namespace amr_interfaces {
namespace placement {

// 0.748 keeps the 0-10 mm stop-short DOCK_ONLY window inside the release
// IK self-collision-free reach band 0.7430-0.7635 m (Native49: 0.764 hit
// front_lidar_link from the 0.755 stance).
constexpr double kDesiredProduct102SlotBaseX = 0.748000000;
constexpr double kDesiredProduct102SlotBaseY = 0.100000000;

// Intermediate values remain available to manipulation's existing local gates.
struct FinalPlacementStance {
  double lateral_offset;
  bool product102_center_slot;
  double direction_radius;
  double base_radius;
  double scale;
  double base_x;
  double base_y;
  double map_x;
  double map_y;
  std::array<double, 3> physical;  // map X/Y in metres and dock yaw in radians
};

inline FinalPlacementStance final_placement_stance(
  int product_id, const std::array<double, 3> & dispatch_dock,
  const std::array<double, 3> & selected_slot)
{
  if (product_id != 101 && product_id != 102 && product_id != 103) {
    throw std::runtime_error("desired placement stance product was unsupported");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(dispatch_dock[i]) || !std::isfinite(selected_slot[i])) {
      throw std::runtime_error("desired placement stance registry input was non-finite");
    }
  }
  // Extracted unchanged from gate6_mass_stage's validated stance calculation.
  constexpr double kDesiredSlotBaseX = 0.520000000;
  constexpr double kDesiredSlotBaseY = -0.580000000;
  // Keep the upper-slot outward bias and the existing radial reach.
  constexpr double kDesiredUpperSlotBaseY = -0.640000000;
  constexpr double kMaxPlacementReleaseRadius = 0.785;
  constexpr double kMaxPlacementAlignmentPositionError = 0.07;
  constexpr double kPlacementReachReserve = 0.005;
  constexpr double kDesiredSlotBaseRadius =
    kMaxPlacementReleaseRadius - kMaxPlacementAlignmentPositionError -
    kPlacementReachReserve;
  const double selected_slot_lateral_offset = selected_slot[1] - dispatch_dock[1];
  if (!std::isfinite(selected_slot_lateral_offset))
    throw std::runtime_error("desired placement stance lateral offset was non-finite");
  const bool product102_center_slot =
    product_id == 102 && selected_slot_lateral_offset == 0.0;
  double desired_slot_direction_x;
  double desired_slot_direction_y;
  if (product102_center_slot) {
    desired_slot_direction_x = kDesiredProduct102SlotBaseX;
    desired_slot_direction_y = kDesiredProduct102SlotBaseY;
  } else if (selected_slot_lateral_offset > 0.0) {
    desired_slot_direction_x = kDesiredSlotBaseX;
    desired_slot_direction_y = kDesiredUpperSlotBaseY;
  } else if (selected_slot_lateral_offset < 0.0) {
    desired_slot_direction_x = kDesiredSlotBaseX;
    desired_slot_direction_y = -kDesiredSlotBaseY;
  } else {
    desired_slot_direction_x = 1.0;
    desired_slot_direction_y = 0.0;
  }
  const double desired_slot_direction_radius =
    std::hypot(desired_slot_direction_x, desired_slot_direction_y);
  const double desired_slot_base_radius = product102_center_slot ?
    desired_slot_direction_radius : kDesiredSlotBaseRadius;
  if (!std::isfinite(desired_slot_direction_radius) || desired_slot_direction_radius <= 0.0 ||
    !std::isfinite(desired_slot_base_radius) || desired_slot_base_radius <= 0.0)
  {
    throw std::runtime_error("desired placement stance radius was invalid");
  }
  const double desired_slot_scale = desired_slot_base_radius / desired_slot_direction_radius;
  const double desired_slot_base_x = desired_slot_direction_x * desired_slot_scale;
  const double desired_slot_base_y = desired_slot_direction_y * desired_slot_scale;
  if (!std::isfinite(desired_slot_scale) || !std::isfinite(desired_slot_base_x) ||
    !std::isfinite(desired_slot_base_y))
  {
    throw std::runtime_error("desired placement stance scaling was non-finite");
  }
  const double dispatch_yaw = dispatch_dock[2];
  const double desired_slot_map_x =
    std::cos(dispatch_yaw) * desired_slot_base_x -
    std::sin(dispatch_yaw) * desired_slot_base_y;
  const double desired_slot_map_y =
    std::sin(dispatch_yaw) * desired_slot_base_x +
    std::cos(dispatch_yaw) * desired_slot_base_y;
  const std::array<double, 3> physical{
    selected_slot[0] - desired_slot_map_x,
    selected_slot[1] - desired_slot_map_y,
    dispatch_yaw};
  if (!std::isfinite(desired_slot_map_x) || !std::isfinite(desired_slot_map_y) ||
    !std::isfinite(physical[0]) || !std::isfinite(physical[1]) || !std::isfinite(physical[2]))
  {
    throw std::runtime_error("desired placement stance map geometry was non-finite");
  }
  return {selected_slot_lateral_offset, product102_center_slot,
    desired_slot_direction_radius, desired_slot_base_radius, desired_slot_scale,
    desired_slot_base_x, desired_slot_base_y, desired_slot_map_x, desired_slot_map_y, physical};
}

}  // namespace placement
}  // namespace amr_interfaces

#endif  // AMR_INTERFACES__FINAL_PLACEMENT_STANCE_HPP_
