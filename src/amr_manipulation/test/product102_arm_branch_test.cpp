// Offline kinematic checks of the Product 102 arm branches against the real
// robot model (xacro URDF + SRDF + KDL). Geometry mirrors gate6_mass_stage.
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "amr_interfaces/final_placement_stance.hpp"
#include "amr_manipulation/product102_arm_branch.hpp"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
const std::array<double, 3> kDispatchDock{-3.4, 0.0, kPi};
const std::array<double, 3> kSlotB{-4.10, 0.0, 0.075};
// Measured Native49 Product 102 pre-grasp joints (joint_states, t=57.6 s).
const std::vector<double> kRecordedPregrasp{
  -0.04118, -0.66399, 0.50327, 0.25182, 0.16569, -0.24858};

std::string run(const std::string & command)
{
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) throw std::runtime_error("popen failed: " + command);
  std::string out;
  char buffer[4096];
  while (fgets(buffer, sizeof(buffer), pipe.get())) out += buffer;
  return out;
}

class Product102ArmBranch : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    const auto share = ament_index_cpp::get_package_share_directory("amr_description");
    const std::string urdf = run(
      "xacro " + share + "/urdf/phase14_mobile_manipulator.urdf.xacro"
      " controller_config:=" + share + "/config/phase14_mobile_manipulator_controllers.yaml"
      " loaded_product:=false factory_attachment:=true"
      " joint_state_topic:=/world/factory_world/model/amr/joint_state");
    const std::string srdf = run("cat " + share + "/config/phase14_mobile_manipulator.srdf");
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      {"robot_description", urdf},
      {"robot_description_semantic", srdf},
      {"robot_description_kinematics.manipulator.kinematics_solver",
        std::string("kdl_kinematics_plugin/KDLKinematicsPlugin")},
      {"robot_description_kinematics.manipulator.kinematics_solver_search_resolution", 0.005},
      {"robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.05}});
    node_ = std::make_shared<rclcpp::Node>("product102_arm_branch_test", options);
    loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    model_ = loader_->getModel();
    ASSERT_TRUE(model_);
    scene_ = std::make_unique<planning_scene::PlanningScene>(model_);
  }

  static void TearDownTestSuite()
  {
    scene_.reset();
    model_.reset();
    loader_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  static moveit::core::RobotState state(const std::vector<double> & arm)
  {
    moveit::core::RobotState st(model_);
    st.setToDefaultValues();
    // Measured open gripper width while carrying/approaching.
    st.setVariablePosition("gripper_finger_joint", 0.035);
    st.setVariablePosition("gripper_right_finger_joint", 0.035);
    st.setJointGroupPositions("manipulator", arm);
    st.update();
    return st;
  }

  static std::string self_contacts(const moveit::core::RobotState & st)
  {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 10;
    collision_detection::CollisionResult result;
    scene_->checkSelfCollision(request, result, st);
    std::string contacts;
    for (const auto & contact : result.contacts) {
      contacts += contact.first.first + "|" + contact.first.second + " ";
    }
    return contacts;
  }

  static std::shared_ptr<rclcpp::Node> node_;
  static std::unique_ptr<robot_model_loader::RobotModelLoader> loader_;
  static moveit::core::RobotModelPtr model_;
  static std::unique_ptr<planning_scene::PlanningScene> scene_;
};

std::shared_ptr<rclcpp::Node> Product102ArmBranch::node_;
std::unique_ptr<robot_model_loader::RobotModelLoader> Product102ArmBranch::loader_;
moveit::core::RobotModelPtr Product102ArmBranch::model_;
std::unique_ptr<planning_scene::PlanningScene> Product102ArmBranch::scene_;

// Native49: the base stopped 9 mm short of the -3.345 stance and the seeded
// release IK put arm_link_2 into front_lidar_link. The DOCK_ONLY admission
// accepts up to 10 mm, and the precision approach stops short of the stance,
// so every stop-short residual in that window must give a clear release.
TEST_F(Product102ArmBranch, ReleaseIkIsSelfCollisionFreeAcrossStopShortAdmission)
{
  const auto stance = amr_interfaces::placement::final_placement_stance(
    102, kDispatchDock, kSlotB).physical;
  const auto * group = model_->getJointModelGroup("manipulator");
  for (const double residual : {0.0, 0.0025, 0.005, 0.0075, 0.010}) {
    SCOPED_TRACE("stop-short residual " + std::to_string(residual));
    const double yaw = stance[2];
    const double rx = stance[0] - residual * std::cos(yaw);
    const double ry = stance[1] - residual * std::sin(yaw);
    const double dx = kSlotB[0] - rx;
    const double dy = kSlotB[1] - ry;
    const double bx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double by = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double bz = kSlotB[2] + 0.020;
    const double radial = std::atan2(by, bx);
    // gate6_mass_stage: map-aligned product yaw, closest pi branch, +1.53.
    const double aligned = std::remainder(-yaw, 2.0 * kPi);
    const double aligned_pi = std::remainder(aligned + kPi, 2.0 * kPi);
    double tcp_yaw = std::abs(aligned) <= std::abs(aligned_pi) ? aligned : aligned_pi;
    tcp_yaw = std::remainder(tcp_yaw + 1.53, 2.0 * kPi);
    constexpr double kQuarterTurn = 0.70710678118654752440;
    geometry_msgs::msg::Pose release;
    release.position.x = bx;
    release.position.y = by;
    release.position.z = bz + 0.080;
    release.orientation.x = -kQuarterTurn * std::sin(tcp_yaw * 0.5);
    release.orientation.y = kQuarterTurn * std::cos(tcp_yaw * 0.5);
    release.orientation.z = kQuarterTurn * std::sin(tcp_yaw * 0.5);
    release.orientation.w = kQuarterTurn * std::cos(tcp_yaw * 0.5);
    auto st = state({-radial, 0.546225552, 0.335934775, -1.693092000, 1.465170000, 2.432600000});
    ASSERT_TRUE(st.setFromIK(group, release, "gripper_tcp", 0.5));
    ASSERT_TRUE(st.satisfiesBounds(group));
    st.update();
    EXPECT_EQ(self_contacts(st), "");
  }
}

// Native49: the Product 102 grasp flipped joints 4 and 6 by ~pi (3.3 rad of
// travel each way). The same TCP pose has an upright branch next to the
// pre-grasp; the grasp seed must select it deterministically.
TEST_F(Product102ArmBranch, GraspSeedSelectsUprightBranchNextToPregrasp)
{
  const auto * group = model_->getJointModelGroup("manipulator");
  const auto pregrasp_state = state(kRecordedPregrasp);
  Eigen::Isometry3d grasp = pregrasp_state.getGlobalLinkTransform("gripper_tcp");
  grasp.translation().z() -= 0.095;  // pre-grasp z 1.000 -> grasp z 0.905
  const auto seed = amr_manipulation::product102_grasp_seed(kRecordedPregrasp[0]);
  for (int attempt = 0; attempt < 5; ++attempt) {
    SCOPED_TRACE("attempt " + std::to_string(attempt));
    auto st = state(std::vector<double>(seed.begin(), seed.end()));
    ASSERT_TRUE(st.setFromIK(group, grasp, "gripper_tcp", 0.5));
    st.update();
    std::vector<double> q;
    st.copyJointGroupPositions(group, q);
    EXPECT_TRUE(amr_manipulation::product102_upright_wrist(q.data()))
      << "j4=" << q[3] << " j6=" << q[5];
    double travel = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
      travel = std::max(travel, std::abs(q[i] - kRecordedPregrasp[i]));
    }
    EXPECT_LE(travel, 0.7);
    EXPECT_EQ(self_contacts(st), "");
  }
}

}  // namespace
