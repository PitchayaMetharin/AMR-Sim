#ifndef AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_
#define AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace amr_manipulation {

// Wrist bound shared by the Product 102 pre-grasp and grasp branches.
constexpr double kProduct102UprightWristBound = 0.5;

inline bool product102_upright_wrist(const double * joints)
{
  return std::isfinite(joints[3]) && std::isfinite(joints[5]) &&
         std::abs(joints[3]) <= kProduct102UprightWristBound &&
         std::abs(joints[5]) <= kProduct102UprightWristBound;
}

// Seed for the Product 102 grasp IK, given the measured pre-grasp joint 1.
// The upright branch sits 0.66 rad from the pre-grasp; the former flipped
// seed did not converge and KDL restarts landed on random wrist-flipped
// branches (Native49: ~pi spin of joints 4 and 6 each way).
inline std::array<double, 6> product102_grasp_seed(double pregrasp_joint_1)
{
  return {pregrasp_joint_1, -0.698, 0.794, -0.406, -0.104, 0.404};
}

// Loaded lift from the upright grasp to the upright pre-grasp, linear in joint
// space (a straight Cartesian lift crosses the j5=0 wrist singularity).
inline std::vector<std::vector<double>> product102_joint_lift(
  const std::vector<double> & grasp, const std::vector<double> & pregrasp,
  std::size_t segments)
{
  if (grasp.size() != pregrasp.size() || grasp.empty() || segments == 0) return {};
  std::vector<std::vector<double>> path;
  path.reserve(segments + 1);
  for (std::size_t step = 0; step <= segments; ++step) {
    const double fraction = static_cast<double>(step) / static_cast<double>(segments);
    std::vector<double> q(grasp.size());
    for (std::size_t i = 0; i < q.size(); ++i) {
      q[i] = grasp[i] + fraction * (pregrasp[i] - grasp[i]);
    }
    path.push_back(std::move(q));
  }
  path.front() = grasp;
  path.back() = pregrasp;
  return path;
}

}  // namespace amr_manipulation

#endif  // AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_
