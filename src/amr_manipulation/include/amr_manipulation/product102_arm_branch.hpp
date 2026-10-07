#ifndef AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_
#define AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_

#include <array>
#include <cmath>

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

}  // namespace amr_manipulation

#endif  // AMR_MANIPULATION__PRODUCT102_ARM_BRANCH_HPP_
