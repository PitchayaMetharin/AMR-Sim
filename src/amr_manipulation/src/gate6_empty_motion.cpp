#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace amr_manipulation {
using namespace std::chrono_literals;

class EmptyMotionGate final : public rclcpp::Node {
 public:
  explicit EmptyMotionGate(const rclcpp::NodeOptions & options)
  : Node("gate6_empty_motion", options) {
    boot_id_ = static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (boot_id_ == 0U) {
      boot_id_ = 1U;
    }
    status_pub_ = create_publisher<amr_interfaces::msg::ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority());
    base_status_sub_ = create_subscription<amr_interfaces::msg::BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic(),
      [this](const amr_interfaces::msg::BaseStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        base_status_ = *message;
        base_status_received_ = std::chrono::steady_clock::now();
        have_base_status_ = true;
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/amr/base/odometry_raw", amr_interfaces::qos::sensor(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        odometry_ = *message;
        odometry_received_ = std::chrono::steady_clock::now();
        have_odometry_ = true;
      });
    set_state(
      amr_interfaces::msg::ManipulatorStatus::STARTING, false,
      "Gate 6 empty-motion acceptance is starting");
    status_timer_ = create_wall_timer(50ms, [this]() { publish_status(); });
    publish_status();
  }

  void set_state(uint8_t state, bool base_motion_allowed, const std::string & detail) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = state;
    base_motion_allowed_ = base_motion_allowed;
    detail_ = detail;
  }

  bool wait_for_motion_permission(std::chrono::seconds timeout) {
    set_state(
      amr_interfaces::msg::ManipulatorStatus::MOVING, false,
      "Arm command inhibited pending fresh READY and 500 ms stationary evidence");

    const auto moving_announced = std::chrono::steady_clock::now();
    const auto deadline = moving_announced + timeout;
    auto stationary_since = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      bool acceptable = false;
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        const bool base_fresh =
          have_base_status_ && now - base_status_received_ <= 200ms;
        const bool odometry_fresh =
          have_odometry_ && now - odometry_received_ <= 200ms;
        const bool base_ready =
          base_fresh && base_status_.valid && base_status_.source_boot_id != 0U &&
          base_status_.sequence != 0U &&
          base_status_.state == amr_interfaces::msg::BaseStatus::READY &&
          base_status_.reason == amr_interfaces::msg::BaseStatus::REASON_READY;
        const auto & twist = odometry_.twist.twist;
        const bool stationary =
          odometry_fresh && std::abs(twist.linear.x) <= 0.01 &&
          std::abs(twist.linear.y) <= 0.01 &&
          std::abs(twist.angular.z) <= 0.01;
        acceptable = base_ready && stationary;
      }
      if (acceptable) {
        if (stationary_since == std::chrono::steady_clock::time_point{}) {
          stationary_since = now;
        }
      } else {
        stationary_since = std::chrono::steady_clock::time_point{};
      }
      if (now - moving_announced >= 400ms &&
        stationary_since != std::chrono::steady_clock::time_point{} &&
        now - stationary_since >= 500ms)
      {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

 private:
  void publish_status() {
    amr_interfaces::msg::ManipulatorStatus message;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      message.header.stamp = now();
      message.source_boot_id = boot_id_;
      message.sequence = ++sequence_;
      message.valid = state_ != amr_interfaces::msg::ManipulatorStatus::FAULT;
      message.state = state_;
      message.base_motion_allowed = base_motion_allowed_;
      message.product_attached = false;
      message.detail = detail_;
    }
    status_pub_->publish(message);
  }

  std::mutex evidence_mutex_;
  amr_interfaces::msg::BaseStatus base_status_;
  nav_msgs::msg::Odometry odometry_;
  std::chrono::steady_clock::time_point base_status_received_{};
  std::chrono::steady_clock::time_point odometry_received_{};
  bool have_base_status_{false};
  bool have_odometry_{false};

  std::mutex status_mutex_;
  uint32_t boot_id_{1U};
  uint32_t sequence_{0U};
  uint8_t state_{amr_interfaces::msg::ManipulatorStatus::STARTING};
  bool base_motion_allowed_{false};
  std::string detail_;
  rclcpp::Publisher<amr_interfaces::msg::ManipulatorStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<amr_interfaces::msg::BaseStatus>::SharedPtr base_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

moveit_msgs::msg::CollisionObject box(
  const std::string & id, const std::array<double, 3> & size,
  const std::array<double, 3> & position)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base_footprint";
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(size.begin(), size.end());
  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace amr_manipulation

int main(int argc, char ** argv) {
  using amr_manipulation::EmptyMotionGate;
  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<EmptyMotionGate>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool passed = false;
  try {
    if (!node->wait_for_motion_permission(8s)) {
      throw std::runtime_error("fresh READY and stationary evidence timed out");
    }

    PlanningSceneInterface planning_scene;
    const std::vector<moveit_msgs::msg::CollisionObject> obstacles{
      amr_manipulation::box("pickup_a_pedestal", {0.40, 0.50, 0.75}, {0.90, 0.0, 0.375}),
      amr_manipulation::box("pickup_a_product_body", {0.30, 0.20, 0.15}, {0.85, 0.0, 0.825}),
      amr_manipulation::box("pickup_a_product_handle", {0.04, 0.10, 0.05}, {0.85, 0.0, 0.925}),
    };
    if (!planning_scene.applyCollisionObjects(obstacles)) {
      throw std::runtime_error("planning-scene collision geometry was rejected");
    }

    MoveGroupInterface arm(node, "manipulator");
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(3);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    const std::vector<double> empty_target{0.20, -1.45, 1.45, 0.0, 0.0, 0.0};
    if (!arm.setJointValueTarget(empty_target)) {
      throw std::runtime_error("empty-motion joint target was rejected");
    }
    MoveGroupInterface::Plan outbound;
    if (arm.plan(outbound) != moveit::core::MoveItErrorCode::SUCCESS ||
      arm.execute(outbound) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      throw std::runtime_error("empty outbound plan or execution failed");
    }

    const std::vector<double> stow_target{0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0};
    if (!arm.setJointValueTarget(stow_target)) {
      throw std::runtime_error("fixed stow joint target was rejected");
    }
    MoveGroupInterface::Plan inbound;
    if (arm.plan(inbound) != moveit::core::MoveItErrorCode::SUCCESS ||
      arm.execute(inbound) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      throw std::runtime_error("stow plan or execution failed");
    }

    const auto current = arm.getCurrentJointValues();
    if (current.size() != stow_target.size()) {
      throw std::runtime_error("current arm joint state has an unexpected size");
    }
    for (std::size_t index = 0; index < current.size(); ++index) {
      if (std::abs(current[index] - stow_target[index]) > 0.01) {
        throw std::runtime_error("fixed stow tolerance was not achieved");
      }
    }
    node->set_state(
      amr_interfaces::msg::ManipulatorStatus::STOWED_EMPTY, true,
      "Gate 6 empty motion passed at 0.2 velocity and acceleration scaling");
    RCLCPP_INFO(node->get_logger(), "GATE 6 EMPTY MOTION: PASS");
    passed = true;
  } catch (const std::exception & error) {
    node->set_state(
      amr_interfaces::msg::ManipulatorStatus::FAULT, false, error.what());
    RCLCPP_ERROR(node->get_logger(), "GATE 6 EMPTY MOTION: FAIL: %s", error.what());
  }

  std::this_thread::sleep_for(300ms);
  rclcpp::shutdown();
  spin_thread.join();
  return passed ? 0 : 1;
}
