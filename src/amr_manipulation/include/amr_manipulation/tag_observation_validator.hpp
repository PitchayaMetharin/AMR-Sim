#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace amr_manipulation {

struct TagObservation {
  std::string family{"tag36h11"};
  std::int32_t id{0};
  std::int32_t hamming{0};
  double receive_age_seconds{0.0};
  double steady_receive_seconds{0.0};
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct TagValidationResult {
  bool accepted{false};
  std::string detail;
};

TagValidationResult validate_tag_observations(
  std::int32_t expected_id,
  const std::vector<TagObservation> & observations);

}  // namespace amr_manipulation
