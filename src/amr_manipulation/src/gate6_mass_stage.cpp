#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "amr_manipulation/attachment_gate.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace amr_manipulation {
using namespace std::chrono_literals;
using SteadyTime = std::chrono::steady_clock::time_point;
constexpr std::array<int, 3> kProductIds{101, 102, 103};

struct ProductSpec {
  int id;
  std::string model;
  double mass_kg;
  std::array<double, 3> size;
  std::array<double, 3> pickup_station;
  std::array<double, 3> pickup_dock;
  std::array<double, 3> pickup_egress;
  double pickup_egress_speed_mps;
  double pickup_egress_time_limit_s;
  double pickup_egress_max_distance_m;
  std::array<double, 3> dispatch_approach;
  std::array<double, 3> dispatch_dock;
  std::array<std::array<double, 3>, 3> dispatch_slots;
  int selected_slot_index;
};

class MassStageNode final : public rclcpp::Node {
 public:
  MassStageNode(const ProductSpec & product, const rclcpp::NodeOptions & options)
  : Node("gate6_mass_stage", options), product_(product) {
    boot_id_ = static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (boot_id_ == 0U) boot_id_ = 1U;

    status_pub_ = create_publisher<amr_interfaces::msg::ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority());
    attach_pub_ = create_publisher<std_msgs::msg::Empty>(
      attachment_topic(product_.id, "attach"), amr_interfaces::qos::state());
    for (std::size_t index = 0; index < kProductIds.size(); ++index) {
      detach_pubs_[index] = create_publisher<std_msgs::msg::Empty>(
        attachment_topic(kProductIds[index], "detach"), amr_interfaces::qos::state());
      state_subs_[index] = create_subscription<std_msgs::msg::String>(
        attachment_topic(kProductIds[index], "state"), amr_interfaces::qos::state(),
        [this, index](const std_msgs::msg::String::SharedPtr message) {
          std::lock_guard<std::mutex> lock(evidence_mutex_);
          attachment_states_[index] = message->data;
          attachment_state_received_[index] = std::chrono::steady_clock::now();
        });
    }
    base_sub_ = create_subscription<amr_interfaces::msg::BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic(),
      [this](const amr_interfaces::msg::BaseStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        base_status_ = *message;
        base_received_ = std::chrono::steady_clock::now();
        have_base_ = true;
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/amr/base/odometry_raw", amr_interfaces::qos::sensor(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        odometry_ = *message;
        odometry_received_ = std::chrono::steady_clock::now();
        have_odometry_ = true;
      });
    product_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/model/" + product_.model + "/pose", amr_interfaces::qos::sensor(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        product_pose_ = *message;
        product_pose_received_ = std::chrono::steady_clock::now();
        have_product_pose_ = true;
      });
    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/amr/simulation/ground_truth/pose", amr_interfaces::qos::sensor(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        robot_pose_ = *message;
        robot_pose_received_ = std::chrono::steady_clock::now();
        have_robot_pose_ = true;
      });
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/amr/base/joint_states", amr_interfaces::qos::sensor(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        joint_states_ = *message;
        joint_states_received_ = std::chrono::steady_clock::now();
        have_joint_states_ = true;
      });
    left_contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/amr/simulation/contacts/left_finger", amr_interfaces::qos::sensor(),
      [this](const ros_gz_interfaces::msg::Contacts::SharedPtr message) {
        record_product_contact(*message, true);
      });
    right_contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/amr/simulation/contacts/right_finger", amr_interfaces::qos::sensor(),
      [this](const ros_gz_interfaces::msg::Contacts::SharedPtr message) {
        record_product_contact(*message, false);
      });
    navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "/amr/mission/navigate_to_pose");
    egress_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(
      this, "/amr/control/dock_egress");
    status_timer_ = create_wall_timer(50ms, [this]() { publish_status(); });
    set_status(amr_interfaces::msg::ManipulatorStatus::STARTING, false, false,
      "Gate 6 mass stage is starting");
    publish_status();
  }

  void set_status(uint8_t state, bool base_allowed, bool attached,
    const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    state_ = state;
    base_allowed_ = base_allowed;
    attached_ = attached;
    detail_ = detail;
  }

  bool wait_for_motion_permission(std::chrono::seconds timeout, bool attached = false) {
    set_status(amr_interfaces::msg::ManipulatorStatus::MOVING, false, attached,
      "Arm command inhibited pending fresh READY and 500 ms stationary evidence");
    const auto announced = std::chrono::steady_clock::now();
    const auto deadline = announced + timeout;
    SteadyTime stationary_since{};
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      bool acceptable = false;
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        const bool base_fresh = have_base_ && now - base_received_ <= 200ms;
        const bool odom_fresh = have_odometry_ && now - odometry_received_ <= 200ms;
        const auto & twist = odometry_.twist.twist;
        acceptable = base_fresh && odom_fresh && base_status_.valid &&
          base_status_.source_boot_id != 0U && base_status_.sequence != 0U &&
          base_status_.state == amr_interfaces::msg::BaseStatus::READY &&
          base_status_.reason == amr_interfaces::msg::BaseStatus::REASON_READY &&
          std::abs(twist.linear.x) <= 0.01 && std::abs(twist.linear.y) <= 0.01 &&
          std::abs(twist.angular.z) <= 0.01;
      }
      stationary_since = acceptable ?
        (stationary_since == SteadyTime{} ? now : stationary_since) : SteadyTime{};
      if (now - announced >= 400ms && stationary_since != SteadyTime{} &&
        now - stationary_since >= 500ms)
      {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool capture_reference_evidence(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        if (have_product_pose_ && have_robot_pose_ &&
          now - product_pose_received_ <= 200ms && now - robot_pose_received_ <= 200ms)
        {
          reference_product_pose_ = product_pose_.pose;
          reference_robot_pose_ = robot_pose_.pose;
          have_reference_ = true;
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool reference_evidence_stable(bool include_product = true) {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!have_reference_ || !have_robot_pose_ ||
      now - robot_pose_received_ > 200ms ||
      (include_product && (!have_product_pose_ || now - product_pose_received_ > 200ms)))
    {
      return false;
    }
    const auto position_error = [](const geometry_msgs::msg::Pose & lhs,
        const geometry_msgs::msg::Pose & rhs) {
        const double dx = lhs.position.x - rhs.position.x;
        const double dy = lhs.position.y - rhs.position.y;
        const double dz = lhs.position.z - rhs.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
      };
    const auto orientation_error = [](const geometry_msgs::msg::Pose & lhs,
        const geometry_msgs::msg::Pose & rhs) {
        const double dot = lhs.orientation.x * rhs.orientation.x +
          lhs.orientation.y * rhs.orientation.y +
          lhs.orientation.z * rhs.orientation.z +
          lhs.orientation.w * rhs.orientation.w;
        return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
      };
    const double robot_position_error = position_error(robot_pose_.pose, reference_robot_pose_);
    const double robot_orientation_error = orientation_error(robot_pose_.pose, reference_robot_pose_);
    const double product_position_error = include_product ?
      position_error(product_pose_.pose, reference_product_pose_) : 0.0;
    const double product_orientation_error = include_product ?
      orientation_error(product_pose_.pose, reference_product_pose_) : 0.0;
    RCLCPP_INFO(get_logger(),
      "Station evidence: base %.4f m / %.4f rad, product %.4f m / %.4f rad",
      robot_position_error, robot_orientation_error,
      product_position_error, product_orientation_error);
    return robot_position_error <= 0.010 && robot_orientation_error <= 0.02 &&
      (!include_product ||
      (product_position_error <= 0.010 && product_orientation_error <= 0.05));
  }

  bool wait_for_bilateral_contact(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        if (left_contact_ != SteadyTime{} && right_contact_ != SteadyTime{} &&
          now - left_contact_ <= 100ms && now - right_contact_ <= 100ms)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  bool grasp_pose_within_tolerance(const geometry_msgs::msg::PoseStamped & tcp) {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!have_product_pose_ || !have_robot_pose_ ||
      now - product_pose_received_ > 200ms || now - robot_pose_received_ > 200ms)
    {
      return false;
    }
    // Gate 6 pickup dock poses have zero yaw.  In the fixed top-grasp
    // transform, the product center is 80 mm below the TCP.
    const double expected_x = robot_pose_.pose.position.x + tcp.pose.position.x;
    const double expected_y = robot_pose_.pose.position.y + tcp.pose.position.y;
    const double expected_z =
      robot_pose_.pose.position.z + tcp.pose.position.z - 0.080;
    const double dx = product_pose_.pose.position.x - expected_x;
    const double dy = product_pose_.pose.position.y - expected_y;
    const double dz = product_pose_.pose.position.z - expected_z;
    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    const auto & q = product_pose_.pose.orientation;
    const double orientation_error = 2.0 * std::acos(std::clamp(std::abs(q.w), 0.0, 1.0));
    RCLCPP_INFO(get_logger(), "Attachment evidence: position %.4f m orientation %.4f rad",
      position_error, orientation_error);
    return position_error <= 0.030 && orientation_error <= 0.15;
  }

  AttachmentEvidence measured_attachment_evidence(
    const geometry_msgs::msg::PoseStamped & tcp) const
  {
    AttachmentEvidence evidence;
    evidence.product_id = product_.id;
    const auto quaternion_multiply = [](
        const geometry_msgs::msg::Quaternion & lhs,
        const geometry_msgs::msg::Quaternion & rhs) {
        geometry_msgs::msg::Quaternion result;
        result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
        result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
        result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
        result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
        return result;
      };
    const auto rotate_vector = [](
        const geometry_msgs::msg::Quaternion & quaternion,
        const std::array<double, 3> & vector) {
        const std::array<double, 3> axis{quaternion.x, quaternion.y, quaternion.z};
        const std::array<double, 3> cross_axis_vector{
          axis[1] * vector[2] - axis[2] * vector[1],
          axis[2] * vector[0] - axis[0] * vector[2],
          axis[0] * vector[1] - axis[1] * vector[0]};
        const std::array<double, 3> twice_cross{
          2.0 * cross_axis_vector[0], 2.0 * cross_axis_vector[1],
          2.0 * cross_axis_vector[2]};
        const std::array<double, 3> axis_cross_twice_cross{
          axis[1] * twice_cross[2] - axis[2] * twice_cross[1],
          axis[2] * twice_cross[0] - axis[0] * twice_cross[2],
          axis[0] * twice_cross[1] - axis[1] * twice_cross[0]};
        return std::array<double, 3>{
          vector[0] + quaternion.w * twice_cross[0] + axis_cross_twice_cross[0],
          vector[1] + quaternion.w * twice_cross[1] + axis_cross_twice_cross[1],
          vector[2] + quaternion.w * twice_cross[2] + axis_cross_twice_cross[2]};
      };
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (have_product_pose_ && have_robot_pose_ &&
        now - product_pose_received_ <= 200ms && now - robot_pose_received_ <= 200ms)
      {
        const auto world_tcp_orientation = quaternion_multiply(
          robot_pose_.pose.orientation, tcp.pose.orientation);
        const auto tcp_offset_world = rotate_vector(
          world_tcp_orientation, {0.080, 0.0, 0.0});
        const auto tcp_position_world = rotate_vector(
          robot_pose_.pose.orientation,
          {tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z});
        const double expected_x = robot_pose_.pose.position.x +
          tcp_position_world[0] + tcp_offset_world[0];
        const double expected_y = robot_pose_.pose.position.y +
          tcp_position_world[1] + tcp_offset_world[1];
        const double expected_z = robot_pose_.pose.position.z +
          tcp_position_world[2] + tcp_offset_world[2];
        const double dx = product_pose_.pose.position.x - expected_x;
        const double dy = product_pose_.pose.position.y - expected_y;
        const double dz = product_pose_.pose.position.z - expected_z;
        evidence.position_error_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        geometry_msgs::msg::Quaternion product_relative_orientation;
        product_relative_orientation.y = -std::sqrt(0.5);
        product_relative_orientation.w = std::sqrt(0.5);
        const auto expected_product_orientation = quaternion_multiply(
          world_tcp_orientation, product_relative_orientation);
        const double orientation_dot =
          product_pose_.pose.orientation.x * expected_product_orientation.x +
          product_pose_.pose.orientation.y * expected_product_orientation.y +
          product_pose_.pose.orientation.z * expected_product_orientation.z +
          product_pose_.pose.orientation.w * expected_product_orientation.w;
        evidence.orientation_error_rad = 2.0 * std::acos(std::clamp(
          std::abs(orientation_dot), 0.0, 1.0));
      } else {
        evidence.position_error_m = std::numeric_limits<double>::infinity();
        evidence.orientation_error_rad = std::numeric_limits<double>::infinity();
      }
      evidence.left_contact = left_contact_ != SteadyTime{} &&
        now - left_contact_ <= 100ms;
      evidence.right_contact = right_contact_ != SteadyTime{} &&
        now - right_contact_ <= 100ms;
      evidence.left_contact_age_s = left_contact_ == SteadyTime{} ?
        std::numeric_limits<double>::infinity() :
        std::chrono::duration<double>(now - left_contact_).count();
      evidence.right_contact_age_s = right_contact_ == SteadyTime{} ?
        std::numeric_limits<double>::infinity() :
        std::chrono::duration<double>(now - right_contact_).count();
    }
    return evidence;
  }

  bool gripper_positions_above(const double threshold, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool proven = false;
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (have_joint_states_ && now - joint_states_received_ <= 200ms) {
        bool left_found = false;
        bool right_found = false;
        double left = 0.0;
        double right = 0.0;
        for (std::size_t i = 0; i < joint_states_.name.size(); ++i) {
          if (i >= joint_states_.position.size()) continue;
          if (joint_states_.name[i] == "gripper_finger_joint") {
            left_found = true;
            left = joint_states_.position[i];
          } else if (joint_states_.name[i] == "gripper_right_finger_joint") {
            right_found = true;
            right = joint_states_.position[i];
          }
        }
          if (left_found && right_found && left > threshold && right > threshold) {
            RCLCPP_INFO(get_logger(),
              "Bilateral gripper stall proven: left %.4f m, right %.4f m", left, right);
            proven = true;
          }
        }
      }
      if (proven) return true;
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool dock_pose_within_tolerance(std::chrono::seconds timeout) {
    constexpr double kDockPositionTolerance = 0.155;
    constexpr double kDockYawTolerance = 0.15;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool proven = false;
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (have_robot_pose_ && now - robot_pose_received_ <= 200ms) {
        const double dx = robot_pose_.pose.position.x - product_.dispatch_dock[0];
        const double dy = robot_pose_.pose.position.y - product_.dispatch_dock[1];
        const double position_error = std::hypot(dx, dy);
        const double yaw = std::atan2(
          2.0 * (robot_pose_.pose.orientation.w * robot_pose_.pose.orientation.z),
          1.0 - 2.0 * (robot_pose_.pose.orientation.z * robot_pose_.pose.orientation.z));
        const double orientation_error = std::abs(std::remainder(
          yaw - product_.dispatch_dock[2], 2.0 * std::acos(-1.0)));
          proven = position_error <= kDockPositionTolerance &&
            orientation_error <= kDockYawTolerance;
        }
      }
      if (proven) return true;
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  double slot_position_error(const std::array<double, 3> & slot) const {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (!have_product_pose_ ||
      std::chrono::steady_clock::now() - product_pose_received_ > 200ms)
    {
      return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(
      std::pow(product_pose_.pose.position.x - slot[0], 2) +
      std::pow(product_pose_.pose.position.y - slot[1], 2) +
      std::pow(product_pose_.pose.position.z - slot[2], 2));
  }

  double selected_slot_position_error() const {
    return slot_position_error(product_.dispatch_slots.at(product_.selected_slot_index));
  }

  double nearest_slot_position_error() const {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & slot : product_.dispatch_slots) {
      nearest = std::min(nearest, slot_position_error(slot));
    }
    return nearest;
  }

  bool latest_robot_pose(geometry_msgs::msg::PoseStamped & pose) const {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (!have_robot_pose_ ||
      std::chrono::steady_clock::now() - robot_pose_received_ > 200ms)
    {
      return false;
    }
    pose = robot_pose_;
    return true;
  }

  bool latest_navigation_feedback_pose(geometry_msgs::msg::PoseStamped & pose) const {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (!navigation_feedback_received_ || navigation_feedback_invalid_ ||
      std::chrono::steady_clock::now() - navigation_feedback_received_wall_ > 5s)
    {
      return false;
    }
    pose = navigation_feedback_pose_;
    return pose.header.frame_id == "map";
  }

  bool latest_product_pose(geometry_msgs::msg::PoseStamped & pose) const {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (!have_product_pose_ ||
      std::chrono::steady_clock::now() - product_pose_received_ > 200ms)
    {
      return false;
    }
    pose = product_pose_;
    return true;
  }

  void reset_navigation_feedback() {
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    navigation_feedback_received_ = false;
    navigation_feedback_invalid_ = false;
    navigation_time_backward_ = false;
    navigation_feedback_received_wall_ = SteadyTime{};
    navigation_time_ns_ = 0;
    navigation_distance_remaining_ = std::numeric_limits<double>::infinity();
    navigation_feedback_pose_ = geometry_msgs::msg::PoseStamped{};
  }

  void record_navigation_feedback(
    const nav2_msgs::action::NavigateToPose::Feedback & feedback)
  {
    const auto received_wall = std::chrono::steady_clock::now();
    const auto navigation_duration = rclcpp::Duration(feedback.navigation_time);
    const auto navigation_time_ns = navigation_duration.nanoseconds();
    const auto & pose = feedback.current_pose;
    const bool pose_valid = pose.header.frame_id == "map" &&
      std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
      std::isfinite(pose.pose.position.z) && std::isfinite(pose.pose.orientation.x) &&
      std::isfinite(pose.pose.orientation.y) && std::isfinite(pose.pose.orientation.z) &&
      std::isfinite(pose.pose.orientation.w);
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    if (navigation_feedback_received_ && navigation_time_ns < navigation_time_ns_)
      navigation_time_backward_ = true;
    if (navigation_time_ns < 0) navigation_time_backward_ = true;
    navigation_feedback_received_ = true;
    navigation_feedback_invalid_ = !pose_valid || !std::isfinite(feedback.distance_remaining);
    navigation_feedback_received_wall_ = received_wall;
    navigation_time_ns_ = navigation_time_ns;
    navigation_distance_remaining_ = feedback.distance_remaining;
    navigation_feedback_pose_ = pose;
  }

  void log_navigation_target(const std::array<double, 3> & target) {
    geometry_msgs::msg::PoseStamped ground_truth;
    const bool have_ground_truth = latest_robot_pose(ground_truth);
    RCLCPP_INFO(
      get_logger(),
      "Navigation target: localized=(%.3f, %.3f, %.3f) ground_truth=(%.3f, %.3f, %.3f)",
      target[0], target[1], target[2],
      have_ground_truth ? ground_truth.pose.position.x : std::numeric_limits<double>::quiet_NaN(),
      have_ground_truth ? ground_truth.pose.position.y : std::numeric_limits<double>::quiet_NaN(),
      have_ground_truth ? ground_truth.pose.position.z : std::numeric_limits<double>::quiet_NaN());
  }

  void log_navigation_terminal(
    const std::array<double, 3> & target,
    rclcpp_action::ResultCode code)
  {
    geometry_msgs::msg::PoseStamped feedback_pose;
    geometry_msgs::msg::PoseStamped ground_truth;
    double distance_remaining = std::numeric_limits<double>::infinity();
    int64_t navigation_time_ns = 0;
    bool have_feedback = false;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      have_feedback = navigation_feedback_received_;
      feedback_pose = navigation_feedback_pose_;
      distance_remaining = navigation_distance_remaining_;
      navigation_time_ns = navigation_time_ns_;
    }
    const bool have_ground_truth = latest_robot_pose(ground_truth);
    const auto yaw_from_pose = [](const geometry_msgs::msg::Pose & pose) {
        return std::atan2(
          2.0 * (pose.orientation.w * pose.orientation.z),
          1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
      };
    const double localized_x = have_feedback ? feedback_pose.pose.position.x :
      std::numeric_limits<double>::quiet_NaN();
    const double localized_y = have_feedback ? feedback_pose.pose.position.y :
      std::numeric_limits<double>::quiet_NaN();
    const double localized_yaw = have_feedback ? yaw_from_pose(feedback_pose.pose) :
      std::numeric_limits<double>::quiet_NaN();
    const double ground_truth_yaw = have_ground_truth ? yaw_from_pose(ground_truth.pose) :
      std::numeric_limits<double>::quiet_NaN();
    const double xy_error = have_feedback ? std::hypot(
      target[0] - localized_x, target[1] - localized_y) :
      std::numeric_limits<double>::infinity();
    const double yaw_error = have_feedback ? std::abs(std::remainder(
      target[2] - localized_yaw, 2.0 * std::acos(-1.0))) :
      std::numeric_limits<double>::infinity();
    RCLCPP_INFO(
      get_logger(),
      "Navigation terminal: target=(%.3f, %.3f, %.3f) code=%d "
      "localized=(%.3f, %.3f, %.3f) xy_error=%.3f yaw_error=%.3f "
      "distance_remaining=%.3f simulation_time=%.3f "
      "ground_truth=(x=%.3f, y=%.3f, yaw=%.3f)",
      target[0], target[1], target[2], static_cast<int>(code),
      localized_x, localized_y, localized_yaw, xy_error, yaw_error,
      distance_remaining, static_cast<double>(navigation_time_ns) * 1e-9,
      have_ground_truth ? ground_truth.pose.position.x : std::numeric_limits<double>::quiet_NaN(),
      have_ground_truth ? ground_truth.pose.position.y : std::numeric_limits<double>::quiet_NaN(),
      ground_truth_yaw);
  }

  bool cancel_navigation_goal(
    const rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr & navigation_client,
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle,
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> & result)
  {
    using Action = nav2_msgs::action::NavigateToPose;
    try {
      auto cancel = navigation_client->async_cancel_goal(goal_handle);
      if (cancel.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Navigation cancellation response timed out");
        return false;
      }
      const auto response = cancel.get();
      if (!response) {
        RCLCPP_ERROR(get_logger(), "Navigation cancellation returned a null response");
        return false;
      }
      const auto & goal_id = goal_handle->get_goal_id();
      const bool listed = std::any_of(
        response->goals_canceling.begin(), response->goals_canceling.end(),
        [&goal_id](const auto & goal_info) { return goal_info.goal_id.uuid == goal_id; });
      if (!listed) {
        RCLCPP_ERROR(get_logger(), "Navigation cancellation did not list the accepted goal");
        return false;
      }
      if (result.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Navigation did not reach terminal CANCELED after cancellation");
        return false;
      }
      const auto terminal = result.get();
      log_navigation_terminal(current_navigation_target_, terminal.code);
      if (terminal.code != rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_ERROR(get_logger(), "Navigation terminal result after cancellation was code %d",
          static_cast<int>(terminal.code));
        return false;
      }
      return true;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_ERROR(get_logger(), "Navigation cancellation found an unknown goal handle");
      return false;
    }
  }

  bool navigate_to(const std::array<double, 3> & target, std::chrono::seconds timeout)
  {
    using Action = nav2_msgs::action::NavigateToPose;
    if (!navigation_client_->wait_for_action_server(5s)) return false;
    reset_navigation_feedback();
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      current_navigation_target_ = target;
    }
    log_navigation_target(target);
    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target[0];
    goal.pose.pose.position.y = target[1];
    goal.pose.pose.orientation.z = std::sin(target[2] * 0.5);
    goal.pose.pose.orientation.w = std::cos(target[2] * 0.5);
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
        const std::shared_ptr<const Action::Feedback> feedback) {
        if (feedback) record_navigation_feedback(*feedback);
      };
    const auto monitoring_started = std::chrono::steady_clock::now();
    auto sent = navigation_client_->async_send_goal(goal, options);
    if (sent.wait_for(5s) != std::future_status::ready) return false;
    auto goal_handle = sent.get();
    if (!goal_handle) return false;
    auto result = navigation_client_->async_get_result(goal_handle);
    const auto simulation_limit_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count();
    while (rclcpp::ok()) {
      if (result.wait_for(50ms) == std::future_status::ready) break;
      bool cancel = false;
      const char * reason = "";
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        const auto wall_now = std::chrono::steady_clock::now();
        if (!navigation_feedback_received_ && wall_now - monitoring_started > 5s) {
          cancel = true;
          reason = "no navigation feedback within 5 wall seconds";
        } else if (navigation_feedback_received_ &&
          wall_now - navigation_feedback_received_wall_ > 5s)
        {
          cancel = true;
          reason = "navigation feedback became stale";
        } else if (navigation_feedback_invalid_) {
          cancel = true;
          reason = "navigation feedback did not contain a valid map TF pose";
        } else if (navigation_time_backward_) {
          cancel = true;
          reason = "navigation time became non-monotonic";
        } else if (navigation_feedback_received_ && navigation_time_ns_ > simulation_limit_ns) {
          cancel = true;
          reason = "simulation navigation time exceeded the limit";
        }
      }
      if (cancel) {
        RCLCPP_ERROR(get_logger(), "Canceling navigation target: %s", reason);
        (void)cancel_navigation_goal(navigation_client_, goal_handle, result);
        return false;
      }
    }
    if (!rclcpp::ok() || result.wait_for(0s) != std::future_status::ready) return false;
    const auto wrapped = result.get();
    log_navigation_terminal(target, wrapped.code);
    bool have_feedback = false;
    bool feedback_invalid = false;
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      have_feedback = navigation_feedback_received_;
      feedback_invalid = navigation_feedback_invalid_ || navigation_time_backward_;
    }
    return have_feedback && !feedback_invalid &&
      wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result != nullptr;
  }

  bool bounded_reverse(
    double distance, const char * label, std::chrono::seconds client_timeout)
  {
    using Action = nav2_msgs::action::BackUp;
    if (!label || !std::isfinite(distance) || distance <= 0.0 ||
      !std::isfinite(product_.pickup_egress_max_distance_m) ||
      distance > product_.pickup_egress_max_distance_m ||
      !std::isfinite(product_.pickup_egress_speed_mps) ||
      !std::isfinite(product_.pickup_egress_time_limit_s))
    {
      RCLCPP_ERROR(
        get_logger(), "%s reverse request is invalid: distance=%.4f max_distance=%.4f",
        label ? label : "Unnamed", distance, product_.pickup_egress_max_distance_m);
      return false;
    }
    if (!egress_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(get_logger(), "%s action server timed out after 3 seconds", label);
      return false;
    }
    Action::Goal goal;
    goal.target.x = distance;
    goal.target.y = 0.0;
    goal.target.z = 0.0;
    goal.speed = static_cast<float>(product_.pickup_egress_speed_mps);
    const auto whole_seconds = static_cast<int32_t>(product_.pickup_egress_time_limit_s);
    goal.time_allowance.sec = whole_seconds;
    goal.time_allowance.nanosec = static_cast<uint32_t>(
      (product_.pickup_egress_time_limit_s - static_cast<double>(whole_seconds)) * 1e9);
    auto accepted = egress_client_->async_send_goal(goal);
    if (accepted.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "%s goal acceptance timed out after 3 seconds", label);
      return false;
    }
    auto goal_handle = accepted.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "%s goal was rejected", label);
      return false;
    }
    // Retain the accepted handle for the full wall-clock wait.  A result that
    // takes longer than the client contract is canceled explicitly and must
    // reach a verified CANCELED terminal state before the stage can fail.
    auto result = egress_client_->async_get_result(goal_handle);
    if (result.wait_for(client_timeout) != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(), "%s result exceeded %lld seconds; canceling accepted goal",
        label,
        static_cast<long long>(client_timeout.count()));
      auto cancel = egress_client_->async_cancel_goal(goal_handle);
      if (cancel.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "%s cancellation response timed out", label);
        return false;
      }
      const auto response = cancel.get();
      if (!response) {
        RCLCPP_ERROR(get_logger(), "%s cancellation returned a null response", label);
        return false;
      }
      const auto & goal_id = goal_handle->get_goal_id();
      const bool listed = std::any_of(
        response->goals_canceling.begin(), response->goals_canceling.end(),
        [&goal_id](const auto & goal_info) { return goal_info.goal_id.uuid == goal_id; });
      if (!listed) {
        RCLCPP_ERROR(get_logger(), "%s cancellation did not list the accepted goal", label);
        return false;
      }
      if (result.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(
          get_logger(), "%s did not reach a terminal result after cancellation", label);
        return false;
      }
      const auto terminal = result.get();
      if (terminal.code != rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_ERROR(
          get_logger(), "%s terminal result after cancellation was code %d, expected CANCELED",
          label,
          static_cast<int>(terminal.code));
      }
      return false;
    }
    const auto wrapped = result.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      RCLCPP_ERROR(
        get_logger(), "%s failed with terminal code %d or a null result",
        label,
        static_cast<int>(wrapped.code));
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "%s SUCCEEDED: requested=%.3f m elapsed=%.3f s",
      label,
      distance,
      static_cast<double>(wrapped.result->total_elapsed_time.sec) +
      static_cast<double>(wrapped.result->total_elapsed_time.nanosec) * 1e-9);
    return true;
  }

  bool dock_egress(std::chrono::seconds client_timeout) {
    const double dx = product_.pickup_egress[0] - product_.pickup_dock[0];
    const double dy = product_.pickup_egress[1] - product_.pickup_dock[1];
    const double distance = std::hypot(dx, dy);
    const double yaw_error = std::abs(std::remainder(
      product_.pickup_egress[2] - product_.pickup_dock[2], 2.0 * std::acos(-1.0)));
    if (!std::isfinite(distance) || distance <= 0.0 ||
      !std::isfinite(yaw_error) || yaw_error > 1e-9)
    {
      RCLCPP_ERROR(
        get_logger(), "Registered pickup egress geometry is invalid: distance=%.4f yaw_error=%.4f",
        distance, yaw_error);
      return false;
    }
    return bounded_reverse(distance, "Dock egress", client_timeout);
  }

  bool request_and_confirm_attachment(std::chrono::seconds timeout) {
    const auto selected = selected_product_index();
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      attachment_states_[selected].clear();
      attachment_state_received_[selected] = SteadyTime{};
    }
    attach_pub_->publish(std_msgs::msg::Empty{});
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        if (attachment_states_[selected] == "attached" &&
          attachment_state_received_[selected] != SteadyTime{})
        {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool request_and_confirm_operational_detachment(std::chrono::seconds timeout) {
    const auto selected = selected_product_index();
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      attachment_states_[selected].clear();
      attachment_state_received_[selected] = SteadyTime{};
    }
    detach_pubs_[selected]->publish(std_msgs::msg::Empty{});
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        if (attachment_states_[selected] == "detached" &&
          attachment_state_received_[selected] != SteadyTime{})
        {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool native_attachment_state_is(const std::string & expected) const {
    const auto selected = selected_product_index();
    std::lock_guard<std::mutex> lock(evidence_mutex_);
    return attachment_states_[selected] == expected &&
      attachment_state_received_[selected] != SteadyTime{};
  }

  bool request_and_confirm_initial_detachment(std::chrono::seconds timeout) {
    {
      std::lock_guard<std::mutex> lock(evidence_mutex_);
      if (initial_detachment_attempted_) return false;
      initial_detachment_attempted_ = true;
      attachment_states_.fill("");
      attachment_state_received_.fill(SteadyTime{});
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_request = SteadyTime{};
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto now = std::chrono::steady_clock::now();
      if (next_request == SteadyTime{} || now >= next_request) {
        for (const auto & publisher : detach_pubs_) {
          publisher->publish(std_msgs::msg::Empty{});
        }
        next_request = now + 200ms;
      }
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        bool all_detached = true;
        for (std::size_t index = 0; index < kProductIds.size(); ++index) {
          all_detached = all_detached && attachment_states_[index] == "detached" &&
            attachment_state_received_[index] != SteadyTime{};
        }
        if (all_detached) return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

 private:
  static std::string attachment_topic(int product_id, const std::string & suffix) {
    return "/amr/simulation/internal/attachment/product_" +
      std::to_string(product_id) + "/" + suffix;
  }

  std::size_t selected_product_index() const {
    const auto found = std::find(kProductIds.begin(), kProductIds.end(), product_.id);
    if (found == kProductIds.end()) throw std::runtime_error("selected product is not configured");
    return static_cast<std::size_t>(found - kProductIds.begin());
  }

  void record_product_contact(const ros_gz_interfaces::msg::Contacts & message, bool left) {
    for (const auto & contact : message.contacts) {
      if (contact.collision1.name.find(product_.model) != std::string::npos ||
        contact.collision2.name.find(product_.model) != std::string::npos)
      {
        std::lock_guard<std::mutex> lock(evidence_mutex_);
        (left ? left_contact_ : right_contact_) = std::chrono::steady_clock::now();
        return;
      }
    }
  }

  void publish_status() {
    amr_interfaces::msg::ManipulatorStatus message;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      message.header.stamp = now();
      message.source_boot_id = boot_id_;
      message.sequence = ++sequence_;
      message.valid = state_ != amr_interfaces::msg::ManipulatorStatus::FAULT;
      message.state = state_;
      message.base_motion_allowed = base_allowed_;
      message.product_attached = attached_;
      message.product_id = attached_ ? std::to_string(product_.id) : "";
      message.detail = detail_;
    }
    status_pub_->publish(message);
  }

  ProductSpec product_;
  mutable std::mutex evidence_mutex_;
  amr_interfaces::msg::BaseStatus base_status_;
  nav_msgs::msg::Odometry odometry_;
  sensor_msgs::msg::JointState joint_states_;
  geometry_msgs::msg::PoseStamped product_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::Pose reference_product_pose_;
  geometry_msgs::msg::Pose reference_robot_pose_;
  SteadyTime base_received_{}, odometry_received_{}, product_pose_received_{};
  SteadyTime robot_pose_received_{}, joint_states_received_{}, left_contact_{}, right_contact_{};
  std::array<SteadyTime, 3> attachment_state_received_{};
  bool have_base_{false}, have_odometry_{false};
  bool have_product_pose_{false}, have_robot_pose_{false}, have_joint_states_{false};
  bool have_reference_{false};
  bool initial_detachment_attempted_{false};
  std::array<std::string, 3> attachment_states_{};
  bool navigation_feedback_received_{false};
  bool navigation_feedback_invalid_{false};
  bool navigation_time_backward_{false};
  SteadyTime navigation_feedback_received_wall_{};
  int64_t navigation_time_ns_{0};
  double navigation_distance_remaining_{std::numeric_limits<double>::infinity()};
  geometry_msgs::msg::PoseStamped navigation_feedback_pose_;
  std::array<double, 3> current_navigation_target_{};

  std::mutex status_mutex_;
  uint32_t boot_id_{1U}, sequence_{0U};
  uint8_t state_{amr_interfaces::msg::ManipulatorStatus::STARTING};
  bool base_allowed_{false}, attached_{false};
  std::string detail_;
  rclcpp::Publisher<amr_interfaces::msg::ManipulatorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr, 3> detach_pubs_;
  std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 3> state_subs_;
  rclcpp::Subscription<amr_interfaces::msg::BaseStatus>::SharedPtr base_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr product_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr left_contact_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr right_contact_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr egress_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

moveit_msgs::msg::CollisionObject box(const std::string & id,
  const std::array<double, 3> & size, const std::array<double, 3> & position,
  const std::string & frame = "base_footprint")
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(size.begin(), size.end());
  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0]; pose.position.y = position[1]; pose.position.z = position[2];
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

bool command_gripper(const std::shared_ptr<rclcpp::Node> & node, double position) {
  using Action = control_msgs::action::GripperCommand;
  const auto started = std::chrono::steady_clock::now();
  auto client = rclcpp_action::create_client<Action>(node, "/gripper_controller/gripper_cmd");
  if (!client->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server timed out after 3 seconds");
    return false;
  }
  Action::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 60.0;
  auto sent = client->async_send_goal(goal);
  if (sent.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal acceptance timed out after 3 seconds");
    return false;
  }
  auto goal_handle = sent.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal was rejected");
    return false;
  }
  auto result = client->async_get_result(goal_handle);
  if (result.wait_for(30s) != std::future_status::ready) {
    RCLCPP_ERROR(
      node->get_logger(), "Gripper result timed out after 30 seconds; canceling accepted goal");
    try {
      auto cancel = client->async_cancel_goal(goal_handle);
      if (cancel.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(
          node->get_logger(), "Gripper cancellation response timed out after 3 seconds");
      } else {
        const auto response = cancel.get();
        if (!response) {
          RCLCPP_ERROR(node->get_logger(), "Gripper cancellation returned a null response");
        } else {
          const auto & goal_id = goal_handle->get_goal_id();
          const bool goal_canceling = std::any_of(
            response->goals_canceling.begin(), response->goals_canceling.end(),
            [&goal_id](const auto & goal_info) {
              return goal_info.goal_id.uuid == goal_id;
            });
          if (!goal_canceling) {
            RCLCPP_ERROR(
              node->get_logger(),
              "Gripper cancellation response did not list the accepted goal as canceling");
          }
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node->get_logger(), "Gripper cancellation request failed: %s", error.what());
    }

    if (result.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Gripper goal did not reach a terminal result within 3 seconds of cancellation");
    } else {
      try {
        const auto terminal = result.get();
        if (terminal.code != rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Gripper goal terminal result after cancellation was code %d, expected CANCELED",
            static_cast<int>(terminal.code));
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          node->get_logger(), "Gripper terminal result confirmation failed: %s", error.what());
      }
    }
    return false;
  }
  const auto wrapped = result.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      node->get_logger(), "Gripper action returned non-success result code %d",
      static_cast<int>(wrapped.code));
    return false;
  }
  if (!wrapped.result) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action succeeded with a null result");
    return false;
  }
  if (!(wrapped.result->reached_goal || wrapped.result->stalled)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Gripper result rejected: requested=%.4f m measured=%.4f m "
      "reached_goal=false stalled=false",
      position, wrapped.result->position);
    return false;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  RCLCPP_INFO(
    node->get_logger(),
    "Gripper command succeeded: requested=%.4f m measured=%.4f m elapsed_wall=%.3f s "
    "reached_goal=%s stalled=%s",
    position, wrapped.result->position, elapsed,
    wrapped.result->reached_goal ? "true" : "false",
    wrapped.result->stalled ? "true" : "false");
  return true;
}

std::array<double, 3> map_point_to_base(
  const std::array<double, 3> & map_point,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  const double yaw = std::atan2(
    2.0 * robot_pose.pose.orientation.w * robot_pose.pose.orientation.z,
    1.0 - 2.0 * robot_pose.pose.orientation.z * robot_pose.pose.orientation.z);
  const double dx = map_point[0] - robot_pose.pose.position.x;
  const double dy = map_point[1] - robot_pose.pose.position.y;
  return {
    std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy,
    map_point[2] - robot_pose.pose.position.z,
  };
}

geometry_msgs::msg::Quaternion inverse_yaw_quaternion(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  const double yaw = std::atan2(
    2.0 * robot_pose.pose.orientation.w * robot_pose.pose.orientation.z,
    1.0 - 2.0 * robot_pose.pose.orientation.z * robot_pose.pose.orientation.z);
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(-yaw * 0.5);
  result.w = std::cos(-yaw * 0.5);
  return result;
}

geometry_msgs::msg::Quaternion top_down_radial_quaternion(const double radial_yaw)
{
  if (!std::isfinite(radial_yaw))
    throw std::runtime_error("placement radial yaw is non-finite");

  // q_z(radial_yaw) * q_y(+pi/2): rotate only about base Z so the TCP stays
  // upright while its wrist branch points toward the placement target.  The
  // held-product transform is q_y(-pi/2), so the product remains upright.
  constexpr double kQuarterTurn = 0.70710678118654752440;
  const double half_yaw = radial_yaw * 0.5;
  geometry_msgs::msg::Quaternion result;
  result.x = -kQuarterTurn * std::sin(half_yaw);
  result.y = kQuarterTurn * std::cos(half_yaw);
  result.z = kQuarterTurn * std::sin(half_yaw);
  result.w = kQuarterTurn * std::cos(half_yaw);
  const double norm = std::sqrt(
    result.x * result.x + result.y * result.y + result.z * result.z + result.w * result.w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon())
    throw std::runtime_error("placement orientation is invalid");
  result.x /= norm;
  result.y /= norm;
  result.z /= norm;
  result.w /= norm;
  return result;
}
}  // namespace amr_manipulation

int main(int argc, char ** argv) {
  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto parameter_node = std::make_shared<rclcpp::Node>("gate6_mass_parameters", options);
  const int configured_id = parameter_node->get_parameter_or<int>("product_id", 101);
  const auto require_string = [&parameter_node](const std::string & name) {
      if (!parameter_node->has_parameter(name)) {
        throw std::runtime_error("missing registry parameter " + name);
      }
      return parameter_node->get_parameter(name).as_string();
    };
  const auto require_double = [&parameter_node](const std::string & name) {
      if (!parameter_node->has_parameter(name)) {
        throw std::runtime_error("missing registry parameter " + name);
      }
      return parameter_node->get_parameter(name).as_double();
    };
  const auto require_int = [&parameter_node](const std::string & name) {
      if (!parameter_node->has_parameter(name)) {
        throw std::runtime_error("missing registry parameter " + name);
      }
      return parameter_node->get_parameter(name).as_int();
    };
  const auto require_array = [&parameter_node](const std::string & name) {
      if (!parameter_node->has_parameter(name)) {
        throw std::runtime_error("missing registry parameter " + name);
      }
      const auto values = parameter_node->get_parameter(name).as_double_array();
      if (values.empty() || std::any_of(values.begin(), values.end(),
        [](double value) { return !std::isfinite(value); }))
      {
        throw std::runtime_error("non-finite registry parameter " + name);
      }
      return values;
    };
  const auto to_pose = [&require_array](const std::string & name) {
      const auto values = require_array(name);
      if (values.size() != 3) throw std::runtime_error("registry pose must have 3 values: " + name);
      return std::array<double, 3>{values[0], values[1], values[2]};
    };
  if (configured_id != 101 && configured_id != 102 && configured_id != 103) {
    throw std::runtime_error("product_id must be 101, 102, or 103");
  }
  const auto product_size_values = require_array("product_size");
  if (product_size_values.size() != 3) throw std::runtime_error("product_size must have 3 values");
  const auto selected_slot_index = require_int("selected_dispatch_slot_index");
  if (selected_slot_index < 0 || selected_slot_index >= 3) {
    throw std::runtime_error("selected dispatch slot index is out of range");
  }
  amr_manipulation::ProductSpec product{
    configured_id,
    require_string("product_model"),
    require_double("product_mass_kg"),
    {product_size_values[0], product_size_values[1], product_size_values[2]},
    to_pose("pickup_station_pose"),
    to_pose("pickup_dock_pose"),
    to_pose("pickup_egress_pose"),
    require_double("dock_egress_speed_mps"),
    require_double("dock_egress_time_limit_s"),
    require_double("dock_egress_max_distance_m"),
    to_pose("dispatch_approach_pose"),
    to_pose("dispatch_dock_pose"),
    {to_pose("dispatch_slot_1_position"), to_pose("dispatch_slot_2_position"),
      to_pose("dispatch_slot_3_position")},
    static_cast<int>(selected_slot_index),
  };
  if (require_string("selected_dispatch_slot_id").empty() ||
    !std::isfinite(product.mass_kg) || product.mass_kg < 0.0 ||
    !std::isfinite(product.pickup_egress_speed_mps) ||
    !std::isfinite(product.pickup_egress_time_limit_s) ||
    !std::isfinite(product.pickup_egress_max_distance_m) ||
    product.pickup_egress_speed_mps <= 0.0 ||
    product.pickup_egress_time_limit_s <= 0.0 ||
    product.pickup_egress_max_distance_m <= 0.0)
  {
    throw std::runtime_error("invalid registry product metadata");
  }
  parameter_node.reset();
  auto node = std::make_shared<amr_manipulation::MassStageNode>(product, options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool passed = false;
  bool product_attached = false;
  try {
    amr_manipulation::AttachmentGate attachment_gate;
    const auto require_motion_permission = [&node, &product_attached]() {
        if (!node->wait_for_motion_permission(8s, product_attached))
          throw std::runtime_error("fresh READY and stationary evidence timed out");
      };
    require_motion_permission();
    if (!node->request_and_confirm_initial_detachment(3s))
      throw std::runtime_error("initial detached state was not confirmed for all products");
    if (!node->capture_reference_evidence(3s))
      throw std::runtime_error("fresh dock and product reference evidence timed out");
    if (!amr_manipulation::command_gripper(node, 0.035))
      throw std::runtime_error("gripper failed to reach the open position");
    if (!node->gripper_positions_above(0.034, 3s))
      throw std::runtime_error("fresh bilateral open gripper positions were not proven");
    if (!node->reference_evidence_stable())
      throw std::runtime_error("dock or product moved while opening the gripper");

    PlanningSceneInterface scene;
    const auto set_pickup_support_collision = [&node, &scene](bool allowed) {
        auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene");
        if (!client->wait_for_service(3s)) return false;
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components =
          moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto future = client->async_send_request(request);
        if (future.wait_for(3s) != std::future_status::ready) return false;

        auto matrix = future.get()->scene.allowed_collision_matrix;
        const auto ensure_entry = [&matrix](const std::string & name) {
            const auto found = std::find(
              matrix.entry_names.begin(), matrix.entry_names.end(), name);
            if (found != matrix.entry_names.end())
              return static_cast<std::size_t>(found - matrix.entry_names.begin());
            const auto size = matrix.entry_names.size();
            for (auto & row : matrix.entry_values) row.enabled.push_back(false);
            matrix.entry_names.push_back(name);
            moveit_msgs::msg::AllowedCollisionEntry row;
            row.enabled.assign(size + 1, false);
            matrix.entry_values.push_back(std::move(row));
            return size;
          };
        if (matrix.entry_names.size() != matrix.entry_values.size()) return false;
        for (const auto & row : matrix.entry_values)
          if (row.enabled.size() != matrix.entry_names.size()) return false;
        const auto held_product = ensure_entry("held_product");
        const auto pickup_pedestal = ensure_entry("pickup_pedestal");
        matrix.entry_values.at(held_product).enabled.at(pickup_pedestal) = allowed;
        matrix.entry_values.at(pickup_pedestal).enabled.at(held_product) = allowed;

        moveit_msgs::msg::PlanningScene update;
        update.is_diff = true;
        update.allowed_collision_matrix = std::move(matrix);
        return scene.applyPlanningScene(update);
      };
    const auto set_held_product_finger_collision = [&node, &scene](bool allowed) {
        auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene");
        if (!client->wait_for_service(3s)) return false;
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components =
          moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto future = client->async_send_request(request);
        if (future.wait_for(3s) != std::future_status::ready) return false;

        auto matrix = future.get()->scene.allowed_collision_matrix;
        const auto ensure_entry = [&matrix](const std::string & name) {
            const auto found = std::find(
              matrix.entry_names.begin(), matrix.entry_names.end(), name);
            if (found != matrix.entry_names.end())
              return static_cast<std::size_t>(found - matrix.entry_names.begin());
            const auto size = matrix.entry_names.size();
            for (auto & row : matrix.entry_values) row.enabled.push_back(false);
            matrix.entry_names.push_back(name);
            moveit_msgs::msg::AllowedCollisionEntry row;
            row.enabled.assign(size + 1, false);
            matrix.entry_values.push_back(std::move(row));
            return size;
          };
        if (matrix.entry_names.size() != matrix.entry_values.size()) return false;
        for (const auto & row : matrix.entry_values)
          if (row.enabled.size() != matrix.entry_names.size()) return false;
        const auto held_product = ensure_entry("held_product");
        const auto left_finger = ensure_entry("gripper_left_finger_link");
        const auto right_finger = ensure_entry("gripper_right_finger_link");
        for (const auto finger : {left_finger, right_finger}) {
          matrix.entry_values.at(held_product).enabled.at(finger) = allowed;
          matrix.entry_values.at(finger).enabled.at(held_product) = allowed;
        }

        moveit_msgs::msg::PlanningScene update;
        update.is_diff = true;
        update.allowed_collision_matrix = std::move(matrix);
        return scene.applyPlanningScene(update);
      };
    auto pickup_handle = amr_manipulation::box(
      "pickup_handle", {0.04, 0.10, 0.05}, {0.85, 0.0, 0.925});
    const std::vector<moveit_msgs::msg::CollisionObject> obstacles{
      amr_manipulation::box("pickup_pedestal", {0.40, 0.50, 0.75}, {0.90, 0.0, 0.375}),
      amr_manipulation::box("pickup_product", product.size, {0.85, 0.0, 0.825}),
      pickup_handle,
    };
    if (!scene.applyCollisionObjects(obstacles))
      throw std::runtime_error("planning-scene pickup geometry was rejected");

    MoveGroupInterface arm(node, "manipulator");
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setPlanningTime(5.0); arm.setNumPlanningAttempts(3);
    arm.setMaxVelocityScalingFactor(0.2); arm.setMaxAccelerationScalingFactor(0.2);

    geometry_msgs::msg::Pose staging;
    staging.position.x = 0.25; staging.position.y = 0.0; staging.position.z = 1.05;
    staging.orientation.y = std::sqrt(0.5);
    staging.orientation.w = std::sqrt(0.5);
    arm.setPoseReferenceFrame("base_footprint");
    // Retreat deterministically over the chassis. A free-space OMPL transit
    // reached the same safe endpoint through a physical station contact that
    // displaced the product before MoveIt detected any modelled collision.
    std::vector<geometry_msgs::msg::Pose> staging_waypoints{staging};
    moveit_msgs::msg::RobotTrajectory staging_trajectory;
    if (arm.computeCartesianPath(
        staging_waypoints, 0.005, 0.0, staging_trajectory, true) < 0.99)
      throw std::runtime_error("over-chassis Cartesian staging path was incomplete");
    MoveGroupInterface::Plan staging_plan;
    staging_plan.trajectory_ = staging_trajectory;
    const auto & staging_joint_names = staging_plan.trajectory_.joint_trajectory.joint_names;
    const auto & staging_points = staging_plan.trajectory_.joint_trajectory.points;
    if (staging_points.empty())
      throw std::runtime_error("over-chassis staging plan was empty");
    const auto staging_joint_index = [&staging_joint_names](const std::string & name) {
        const auto found = std::find(staging_joint_names.begin(), staging_joint_names.end(), name);
        if (found == staging_joint_names.end())
          throw std::runtime_error("staging plan omitted " + name);
        return static_cast<std::size_t>(found - staging_joint_names.begin());
      };
    const auto wrist_4_index = staging_joint_index("arm_joint_4");
    const auto wrist_6_index = staging_joint_index("arm_joint_6");
    double wrist_4_path_max = 0.0;
    double wrist_6_path_max = 0.0;
    for (const auto & point : staging_points) {
      wrist_4_path_max = std::max(wrist_4_path_max, std::abs(point.positions.at(wrist_4_index)));
      wrist_6_path_max = std::max(wrist_6_path_max, std::abs(point.positions.at(wrist_6_index)));
    }
    RCLCPP_INFO(
      node->get_logger(), "Staging wrist path max: joint 4 %.4f rad, joint 6 %.4f rad",
      wrist_4_path_max, wrist_6_path_max);
    if (wrist_4_path_max > 0.5 || wrist_6_path_max > 0.5)
      throw std::runtime_error("unsafe wrist-flipped staging branch rejected");
    require_motion_permission();
    if (arm.execute(staging_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("over-chassis staging execution failed");
    if (!node->reference_evidence_stable())
      throw std::runtime_error("dock or product moved during over-chassis staging");

    geometry_msgs::msg::Pose pregrasp = staging;
    pregrasp.position.x = 0.85; pregrasp.position.z = 1.00;
    const std::vector<double> pregrasp_seed{
      -0.000032311, -0.760907950, 0.661511204,
      0.000037514, 0.099207379, -0.000017083};
    auto pregrasp_ik_state = arm.getCurrentState(3.0);
    if (!pregrasp_ik_state)
      throw std::runtime_error("fresh MoveIt state was unavailable for pre-grasp IK preflight");
    const auto * pregrasp_manipulator_group =
      pregrasp_ik_state->getJointModelGroup("manipulator");
    const std::vector<std::string> expected_pregrasp_joint_names{
      "arm_joint_1", "arm_joint_2", "arm_joint_3",
      "arm_joint_4", "arm_joint_5", "arm_joint_6"};
    if (!pregrasp_manipulator_group ||
      pregrasp_manipulator_group->getVariableNames() != expected_pregrasp_joint_names ||
      pregrasp_manipulator_group->getVariableCount() != pregrasp_seed.size())
    {
      throw std::runtime_error("pre-grasp IK manipulator joint order was invalid");
    }
    const auto finite_pregrasp_joint_values = [](const std::vector<double> & values) {
        return !values.empty() && std::all_of(values.begin(), values.end(),
          [](double value) { return std::isfinite(value); });
      };
    if (!finite_pregrasp_joint_values(pregrasp_seed))
      throw std::runtime_error("pre-grasp IK seed was invalid");
    pregrasp_ik_state->setJointGroupPositions(
      pregrasp_manipulator_group, pregrasp_seed);
    pregrasp_ik_state->update();
    if (!pregrasp_ik_state->setFromIK(
        pregrasp_manipulator_group, pregrasp, "gripper_tcp", 0.5) ||
      !pregrasp_ik_state->satisfiesBounds(pregrasp_manipulator_group))
    {
      throw std::runtime_error("exact seeded pre-grasp IK preflight failed");
    }
    std::vector<double> pregrasp_ik_solution;
    pregrasp_ik_state->copyJointGroupPositions(
      pregrasp_manipulator_group, pregrasp_ik_solution);
    if (!finite_pregrasp_joint_values(pregrasp_ik_solution))
      throw std::runtime_error("pre-grasp IK preflight returned invalid joints");
    arm.setStartStateToCurrentState();
    if (!arm.setJointValueTarget(pregrasp_ik_solution))
      throw std::runtime_error("exact pre-grasp joint target was rejected");
    MoveGroupInterface::Plan pregrasp_plan;
    if (arm.plan(pregrasp_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("collision-free pre-grasp plan failed");
    const auto & pregrasp_joint_names = pregrasp_plan.trajectory_.joint_trajectory.joint_names;
    const auto & pregrasp_points = pregrasp_plan.trajectory_.joint_trajectory.points;
    const auto pregrasp_joint_index = [&pregrasp_joint_names](const std::string & name) {
        const auto found = std::find(pregrasp_joint_names.begin(), pregrasp_joint_names.end(), name);
        if (found == pregrasp_joint_names.end())
          throw std::runtime_error("pre-grasp plan omitted " + name);
        return static_cast<std::size_t>(found - pregrasp_joint_names.begin());
      };
    const auto pregrasp_wrist_4 = pregrasp_joint_index("arm_joint_4");
    const auto pregrasp_wrist_6 = pregrasp_joint_index("arm_joint_6");
    for (const auto & point : pregrasp_points) {
      if (std::abs(point.positions.at(pregrasp_wrist_4)) > 0.5 ||
        std::abs(point.positions.at(pregrasp_wrist_6)) > 0.5)
      {
        throw std::runtime_error("unsafe wrist-flipped pre-grasp branch rejected");
      }
    }
    require_motion_permission();
    if (arm.execute(pregrasp_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("pre-grasp execution failed");
    if (!node->reference_evidence_stable())
      throw std::runtime_error("dock or product moved during pre-grasp motion");

    // The handle must constrain the free-space pre-grasp plan.  Remove only
    // this intentional-contact geometry before the straight grasp approach;
    // the product body and pedestal remain collision obstacles.
    pickup_handle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene.applyCollisionObject(pickup_handle))
      throw std::runtime_error("planning-scene handle removal was rejected");

    geometry_msgs::msg::Pose grasp = pregrasp;
    grasp.position.z = 0.905;
    std::vector<geometry_msgs::msg::Pose> waypoints{grasp};
    moveit_msgs::msg::RobotTrajectory approach;
    if (arm.computeCartesianPath(waypoints, 0.005, 0.0, approach, true) < 0.99)
      throw std::runtime_error("Cartesian grasp approach was incomplete");
    MoveGroupInterface::Plan approach_plan;
    approach_plan.trajectory_ = approach;
    require_motion_permission();
    if (arm.execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Cartesian grasp approach execution failed");
    if (!node->reference_evidence_stable())
      throw std::runtime_error("dock or product moved during grasp approach");
    require_motion_permission();
    if (!amr_manipulation::command_gripper(node, 0.020))
      throw std::runtime_error("gripper close command failed");
    if (!node->gripper_positions_above(0.020, 3s))
      throw std::runtime_error("fresh bilateral contact gripper positions were not proven");
    if (!node->wait_for_bilateral_contact(3s))
      throw std::runtime_error("fresh bilateral product contact was not proven");
    if (!node->reference_evidence_stable())
      throw std::runtime_error("dock or product moved beyond contact tolerance");
    const auto attachment_evidence = node->measured_attachment_evidence(
      arm.getCurrentPose("gripper_tcp"));
    const auto attach_decision = attachment_gate.evaluate_attach(attachment_evidence);
    if (attach_decision != amr_manipulation::AttachmentDecision::ACCEPT)
      throw std::runtime_error("attachment evidence rejected before native attach command");
    if (!node->request_and_confirm_attachment(3s))
      throw std::runtime_error("Gazebo attachment confirmation timed out");
    if (!attachment_gate.confirm_attached(product.id))
      throw std::runtime_error("fresh native attachment confirmation was not accepted");
    product_attached = true;

    scene.removeCollisionObjects({"pickup_product"});
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "gripper_left_finger_link";
    attached.touch_links = {"gripper_left_finger_link", "gripper_right_finger_link", "gripper_base_link"};
    attached.object = amr_manipulation::box(
      "held_product", product.size, {0.080, 0.0, 0.0}, "gripper_tcp");
    attached.object.primitive_poses.front().orientation.y = -std::sqrt(0.5);
    attached.object.primitive_poses.front().orientation.w = std::sqrt(0.5);
    shape_msgs::msg::SolidPrimitive handle;
    handle.type = shape_msgs::msg::SolidPrimitive::BOX;
    handle.dimensions = {0.04, 0.10, 0.05};
    geometry_msgs::msg::Pose handle_pose;
    handle_pose.position.x = -0.020;
    handle_pose.orientation.y = -std::sqrt(0.5);
    handle_pose.orientation.w = std::sqrt(0.5);
    attached.object.primitives.push_back(handle);
    attached.object.primitive_poses.push_back(handle_pose);
    if (!scene.applyAttachedCollisionObject(attached))
      throw std::runtime_error("MoveIt attached collision object was rejected");

    // The product begins the lift in expected contact with its support.  Allow
    // only this object pair while moving straight upward, then restore normal
    // collision checking before any free-space loaded motion.
    if (!set_pickup_support_collision(true))
      throw std::runtime_error("temporary pickup support collision allowance was rejected");
    auto lift_checkpoint = grasp;
    lift_checkpoint.position.z += 0.080;
    auto clearance_retreat = pregrasp;
    try {
      // Keep the checkpoint in the same collision-checked Cartesian path as
      // the final clearance retreat.  Stopping at the checkpoint leaves the
      // held product in the known self-collision boundary.
      std::vector<geometry_msgs::msg::Pose> retreat_waypoints{
        lift_checkpoint, clearance_retreat};
      moveit_msgs::msg::RobotTrajectory retreat_trajectory;
      if (arm.computeCartesianPath(
          retreat_waypoints, 0.005, 0.0, retreat_trajectory, true) < 0.99)
        throw std::runtime_error("continuous Cartesian retreat was incomplete");
      MoveGroupInterface::Plan retreat_plan;
      retreat_plan.trajectory_ = retreat_trajectory;
      require_motion_permission();
      if (arm.execute(retreat_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("continuous Cartesian retreat execution failed");
    } catch (...) {
      if (!set_pickup_support_collision(false))
        RCLCPP_ERROR(node->get_logger(), "Failed to restore pickup support collision checking");
      throw;
    }
    if (!set_pickup_support_collision(false))
      throw std::runtime_error("pickup support collision checking was not restored");
    if (!node->reference_evidence_stable(false))
      throw std::runtime_error("dock moved during loaded retreat");
    geometry_msgs::msg::PoseStamped retreat_product_pose;
    if (!node->latest_product_pose(retreat_product_pose) ||
      !node->native_attachment_state_is("attached"))
    {
      throw std::runtime_error("product evidence was not stable after loaded retreat");
    }
    const auto retreat_attachment = node->measured_attachment_evidence(
      arm.getCurrentPose("gripper_tcp"));
    if (retreat_attachment.position_error_m > 0.030 ||
      retreat_attachment.orientation_error_rad > 0.15)
    {
      throw std::runtime_error("product attachment evidence was not stable after loaded retreat");
    }

    // Validate a fresh state after the complete retreat, before asking MoveIt
    // to plan any free-space loaded motion.  Every reported contact is kept
    // in the log so an invalid state fails closed with actionable evidence.
    auto current_state = arm.getCurrentState(3.0);
    if (!current_state)
      throw std::runtime_error("fresh MoveIt current state was unavailable");
    auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity");
    if (!validity_client->wait_for_service(3s))
      throw std::runtime_error("/check_state_validity service was unavailable");
    auto validity_request =
      std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    moveit::core::robotStateToRobotStateMsg(*current_state, validity_request->robot_state);
    validity_request->group_name = "manipulator";
    auto validity_future = validity_client->async_send_request(validity_request);
    if (validity_future.wait_for(3s) != std::future_status::ready)
      throw std::runtime_error("/check_state_validity request timed out");
    const auto validity_response = validity_future.get();
    if (!validity_response)
      throw std::runtime_error("/check_state_validity returned no response");
    for (const auto & contact : validity_response->contacts) {
      RCLCPP_WARN(
        node->get_logger(), "MoveIt state contact: %s <-> %s",
        contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
    }
    if (!validity_response->valid)
      throw std::runtime_error("fresh loaded retreat state is invalid");

    const std::vector<double> stow{0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0};
    if (!arm.setJointValueTarget(stow)) throw std::runtime_error("loaded stow target was rejected");
    arm.setStartStateToCurrentState();
    MoveGroupInterface::Plan stow_plan;
    if (arm.plan(stow_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("loaded stow planning failed");
    require_motion_permission();
    if (arm.execute(stow_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("loaded stow execution failed");
    if (!node->reference_evidence_stable(false))
      throw std::runtime_error("dock moved during loaded stow");
    const auto current = arm.getCurrentJointValues();
    if (current.size() != stow.size()) throw std::runtime_error("loaded stow joint state is incomplete");
    for (std::size_t i = 0; i < stow.size(); ++i)
      if (std::abs(current[i] - stow[i]) > 0.01)
        throw std::runtime_error("loaded stow tolerance was not achieved");

    node->set_status(amr_interfaces::msg::ManipulatorStatus::STOWED_LOADED,
      true, true, "Gate 6 " + std::to_string(product.mass_kg) + " kg grasp and loaded stow passed");
    std::this_thread::sleep_for(400ms);

    // A dispatch request while still at pickup is a mandatory negative test.
    // The gate must reject it before the internal native detach publisher is
    // touched, and the simulator must still report the product attached.
    const auto pickup_detach_decision = attachment_gate.evaluate_detach(
      product.id, node->nearest_slot_position_error());
    if (pickup_detach_decision !=
      amr_manipulation::AttachmentDecision::DISPATCH_POSE_OUT_OF_TOLERANCE ||
      node->selected_slot_position_error() <= 0.030 ||
      !node->native_attachment_state_is("attached"))
    {
      throw std::runtime_error("out-of-dispatch detachment rejection check failed");
    }
    RCLCPP_INFO(node->get_logger(),
      "Out-of-dispatch detachment rejection: PASS (native state remains attached)");

    if (!node->reference_evidence_stable(false) ||
      !node->native_attachment_state_is("attached"))
    {
      throw std::runtime_error("fresh pickup dock stability or attachment evidence was lost before egress");
    }
    if (!node->dock_egress(65s))
      throw std::runtime_error("pickup dock egress failed");
    if (!node->native_attachment_state_is("attached"))
      throw std::runtime_error("product attachment was not retained after pickup dock egress");

    // Pickup collision objects are expressed in base_footprint.  They must be
    // removed before transport or they would move with the AMR.
    scene.removeCollisionObjects({"pickup_pedestal", "pickup_product", "pickup_handle"});
    // The station approach is a second bounded reverse leg.  The registered
    // pickup poses are collinear, so this preserves the base heading instead
    // of asking RPP to reverse along a short path while turning 180 degrees.
    const double dock_to_egress_dx = product.pickup_egress[0] - product.pickup_dock[0];
    const double dock_to_egress_dy = product.pickup_egress[1] - product.pickup_dock[1];
    const double dock_to_approach_dx = product.pickup_station[0] - product.pickup_dock[0];
    const double dock_to_approach_dy = product.pickup_station[1] - product.pickup_dock[1];
    const double egress_to_approach_dx = product.pickup_station[0] - product.pickup_egress[0];
    const double egress_to_approach_dy = product.pickup_station[1] - product.pickup_egress[1];
    const double pickup_approach_distance = std::hypot(
      egress_to_approach_dx, egress_to_approach_dy);
    const double dock_to_approach_distance_sq =
      dock_to_approach_dx * dock_to_approach_dx + dock_to_approach_dy * dock_to_approach_dy;
    const double collinearity =
      dock_to_egress_dx * dock_to_approach_dy - dock_to_egress_dy * dock_to_approach_dx;
    const double ordering =
      dock_to_egress_dx * dock_to_approach_dx + dock_to_egress_dy * dock_to_approach_dy;
    const double pickup_approach_yaw_error = std::abs(std::remainder(
      product.pickup_station[2] - product.pickup_egress[2], 2.0 * std::acos(-1.0)));
    if (!std::isfinite(product.pickup_station[0]) ||
      !std::isfinite(product.pickup_station[1]) ||
      !std::isfinite(product.pickup_station[2]) ||
      !std::isfinite(product.pickup_egress[0]) ||
      !std::isfinite(product.pickup_egress[1]) ||
      !std::isfinite(pickup_approach_distance) || pickup_approach_distance <= 0.0 ||
      pickup_approach_distance > product.pickup_egress_max_distance_m ||
      !std::isfinite(dock_to_approach_distance_sq) ||
      std::abs(collinearity) > 1e-9 ||
      !(0.0 < ordering && ordering < dock_to_approach_distance_sq) ||
      !std::isfinite(pickup_approach_yaw_error) || pickup_approach_yaw_error > 1e-9)
    {
      throw std::runtime_error("pickup station reverse geometry was invalid");
    }
    if (!node->bounded_reverse(
        pickup_approach_distance, "Pickup approach reverse", 65s))
      throw std::runtime_error("pickup station reverse failed");
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed after pickup station reverse");
    if (!node->navigate_to(product.pickup_station, 120s))
      throw std::runtime_error("navigation to pickup station failed");

    geometry_msgs::msg::PoseStamped pickup_station_achieved;
    if (!node->latest_navigation_feedback_pose(pickup_station_achieved))
      throw std::runtime_error("fresh localized pickup station terminal pose was unavailable");
    const double pickup_station_xy_error = std::hypot(
      pickup_station_achieved.pose.position.x - product.pickup_station[0],
      pickup_station_achieved.pose.position.y - product.pickup_station[1]);
    const double pickup_station_yaw = std::atan2(
      2.0 * pickup_station_achieved.pose.orientation.w * pickup_station_achieved.pose.orientation.z,
      1.0 - 2.0 * pickup_station_achieved.pose.orientation.z *
      pickup_station_achieved.pose.orientation.z);
    const double pickup_station_yaw_error = std::abs(std::remainder(
      pickup_station_yaw - product.pickup_station[2], 2.0 * std::acos(-1.0)));
    if (!std::isfinite(pickup_station_xy_error) ||
      !std::isfinite(pickup_station_yaw_error) ||
      pickup_station_xy_error > 0.07 || pickup_station_yaw_error > 0.15)
    {
      throw std::runtime_error("pickup station localized terminal pose was out of tolerance");
    }
    geometry_msgs::msg::PoseStamped dispatch_translation_start;
    if (!node->latest_navigation_feedback_pose(dispatch_translation_start))
      throw std::runtime_error("fresh localized pose was unavailable before dispatch translation");
    const double dispatch_translation_heading = std::atan2(
      product.dispatch_approach[1] - dispatch_translation_start.pose.position.y,
      product.dispatch_approach[0] - dispatch_translation_start.pose.position.x);
    if (!std::isfinite(dispatch_translation_heading))
      throw std::runtime_error("dispatch translation bearing was non-finite");
    const std::array<double, 3> dispatch_translation_target{
      product.dispatch_approach[0], product.dispatch_approach[1],
      dispatch_translation_heading};
    if (!node->navigate_to(dispatch_translation_target, 120s))
      throw std::runtime_error("navigation to dispatch approach translation failed");
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed after dispatch approach translation");

    geometry_msgs::msg::PoseStamped dispatch_heading_start;
    if (!node->latest_navigation_feedback_pose(dispatch_heading_start))
      throw std::runtime_error("fresh localized pose was unavailable before dispatch heading");
    const std::array<double, 3> dispatch_heading_target{
      dispatch_heading_start.pose.position.x,
      dispatch_heading_start.pose.position.y,
      product.dispatch_approach[2]};
    if (!node->navigate_to(dispatch_heading_target, 120s))
      throw std::runtime_error("navigation to dispatch approach heading failed");
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed after dispatch approach heading");
    geometry_msgs::msg::PoseStamped dispatch_dock_bias_ground_truth;
    geometry_msgs::msg::PoseStamped dispatch_dock_bias_localized;
    if (!node->latest_robot_pose(dispatch_dock_bias_ground_truth) ||
      !node->latest_navigation_feedback_pose(dispatch_dock_bias_localized))
    {
      throw std::runtime_error("fresh dispatch dock bias evidence was unavailable");
    }
    const double dispatch_dock_bias_x =
      dispatch_dock_bias_ground_truth.pose.position.x - dispatch_dock_bias_localized.pose.position.x;
    const double dispatch_dock_bias_y =
      dispatch_dock_bias_ground_truth.pose.position.y - dispatch_dock_bias_localized.pose.position.y;
    if (!std::isfinite(dispatch_dock_bias_x) || !std::isfinite(dispatch_dock_bias_y))
      throw std::runtime_error("dispatch dock localization bias was non-finite");
    const std::array<double, 3> dispatch_dock_corrected_target{
      product.dispatch_dock[0] - dispatch_dock_bias_x,
      product.dispatch_dock[1] - dispatch_dock_bias_y,
      product.dispatch_dock[2]};
    if (!node->navigate_to(dispatch_dock_corrected_target, 120s))
      throw std::runtime_error("navigation to dispatch dock failed");
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed after dispatch dock");
    if (!node->dock_pose_within_tolerance(5s))
      throw std::runtime_error("fresh dispatch dock ground-truth pose was out of tolerance");
    geometry_msgs::msg::PoseStamped dock_alignment_ground_truth;
    geometry_msgs::msg::PoseStamped dock_alignment_localized;
    if (!node->latest_robot_pose(dock_alignment_ground_truth) ||
      !node->latest_navigation_feedback_pose(dock_alignment_localized))
    {
      throw std::runtime_error("fresh dock ground-truth or localized pose was unavailable");
    }

    // The registered dock is the required reference.  Align only enough to
    // put the selected slot at the proven reachable base-frame offset; do not
    // add another registry pose or silently exceed the bounded segment cap.
    const auto selected_slot = product.dispatch_slots.at(product.selected_slot_index);
    // This radial stance is the nearest production-KDL branch that also
    // clears the AMR chassis with the held product at the release height.
    // Reach it through bounded navigation segments; each segment remains at
    // or below the existing 0.15 m alignment bound.
    constexpr double kDesiredSlotBaseX = 0.520000000;
    constexpr double kDesiredSlotBaseY = -0.580000000;
    constexpr double kMaxPlacementAlignmentSegmentDisplacement = 0.15;
    constexpr double kMaxPlacementAlignmentTotalDisplacement = 0.35;
    constexpr double kMaxPlacementAlignmentPositionError = 0.07;
    constexpr double kMaxPlacementAlignmentYawError = 0.15;
    constexpr double kMaxPlacementReleaseRadius = 0.785;
    constexpr double kPlacementReachReserve = 0.005;
    constexpr double kDesiredSlotBaseRadius =
      kMaxPlacementReleaseRadius - kMaxPlacementAlignmentPositionError -
      kPlacementReachReserve;
    // Keep the unchanged 0.15 rad acceptance envelope, but command the final
    // same-position heading goal 0.03 rad inside it so Nav2 cannot accept the
    // already-near-heading state without making the required rotation.
    constexpr double kFinalHeadingGoalMargin = 0.03;
    // Leave the existing 0.07 m Nav2 terminal tolerance inside the 0.15 m
    // achieved-segment bound.  Translation goals use their travel bearing so
    // MPPI does not fight the short diagonal path; a final same-position goal
    // then rotates to the approved dock heading.  This changes only goal
    // sequencing, not controller parameters or safety limits.
    constexpr double kMaxPlacementCommandDisplacement =
      kMaxPlacementAlignmentSegmentDisplacement - kMaxPlacementAlignmentPositionError;
    const double desired_slot_direction_radius =
      std::hypot(kDesiredSlotBaseX, kDesiredSlotBaseY);
    if (!std::isfinite(desired_slot_direction_radius) || desired_slot_direction_radius <= 0.0 ||
      !std::isfinite(kDesiredSlotBaseRadius) || kDesiredSlotBaseRadius <= 0.0)
    {
      throw std::runtime_error("desired placement stance radius was invalid");
    }
    const double desired_slot_scale = kDesiredSlotBaseRadius / desired_slot_direction_radius;
    const double desired_slot_base_x = kDesiredSlotBaseX * desired_slot_scale;
    const double desired_slot_base_y = kDesiredSlotBaseY * desired_slot_scale;
    if (!std::isfinite(desired_slot_scale) || !std::isfinite(desired_slot_base_x) ||
      !std::isfinite(desired_slot_base_y))
    {
      throw std::runtime_error("desired placement stance scaling was non-finite");
    }
    const double dispatch_yaw = product.dispatch_dock[2];
    const double desired_slot_map_x =
      std::cos(dispatch_yaw) * desired_slot_base_x -
      std::sin(dispatch_yaw) * desired_slot_base_y;
    const double desired_slot_map_y =
      std::sin(dispatch_yaw) * desired_slot_base_x +
      std::cos(dispatch_yaw) * desired_slot_base_y;
    const std::array<double, 3> placement_alignment_physical{
      selected_slot[0] - desired_slot_map_x,
      selected_slot[1] - desired_slot_map_y,
      dispatch_yaw};
    const auto yaw_from_pose = [](const geometry_msgs::msg::Pose & pose) {
        return std::atan2(
          2.0 * pose.orientation.w * pose.orientation.z,
          1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
      };
    const auto wrap_yaw = [](double value) {
        return std::remainder(value, 2.0 * std::acos(-1.0));
      };
    const double dock_ground_truth_yaw = yaw_from_pose(dock_alignment_ground_truth.pose);
    const double dock_localized_yaw = yaw_from_pose(dock_alignment_localized.pose);
    const double localization_bias_x =
      dock_alignment_ground_truth.pose.position.x - dock_alignment_localized.pose.position.x;
    const double localization_bias_y =
      dock_alignment_ground_truth.pose.position.y - dock_alignment_localized.pose.position.y;
    const double localization_bias_yaw = wrap_yaw(dock_ground_truth_yaw - dock_localized_yaw);
    const std::array<double, 3> placement_alignment{
      placement_alignment_physical[0] - localization_bias_x,
      placement_alignment_physical[1] - localization_bias_y,
      wrap_yaw(placement_alignment_physical[2] - localization_bias_yaw)};
    const double alignment_dx =
      placement_alignment_physical[0] - dock_alignment_ground_truth.pose.position.x;
    const double alignment_dy =
      placement_alignment_physical[1] - dock_alignment_ground_truth.pose.position.y;
    const double alignment_displacement = std::hypot(alignment_dx, alignment_dy);
    const auto alignment_segments = static_cast<std::size_t>(
      std::ceil(alignment_displacement / kMaxPlacementCommandDisplacement));
    const bool alignment_geometry_finite =
      std::isfinite(dispatch_yaw) &&
      std::isfinite(desired_slot_map_x) && std::isfinite(desired_slot_map_y) &&
      std::isfinite(placement_alignment_physical[0]) &&
      std::isfinite(placement_alignment_physical[1]) &&
      std::isfinite(placement_alignment_physical[2]) &&
      std::isfinite(placement_alignment[0]) && std::isfinite(placement_alignment[1]) &&
      std::isfinite(placement_alignment[2]) &&
      std::isfinite(alignment_dx) && std::isfinite(alignment_dy) &&
      std::isfinite(alignment_displacement) &&
      std::isfinite(localization_bias_x) && std::isfinite(localization_bias_y) &&
      std::isfinite(localization_bias_yaw) &&
      alignment_segments > 0 && alignment_segments <= 8;
    if (!alignment_geometry_finite || alignment_displacement <= 0.0 ||
      alignment_displacement > kMaxPlacementAlignmentTotalDisplacement)
    {
      throw std::runtime_error(
              "derived dispatch placement alignment was non-finite or exceeded bounded total motion");
    }
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("loaded-stowed attachment proof failed before placement alignment");
    RCLCPP_INFO(
      node->get_logger(),
      "Dispatch placement alignment: physical=(%.3f, %.3f, %.3f) command=(%.3f, %.3f, %.3f) "
      "segments=%zu total_ground_truth=%.4f m bias=(%.3f, %.3f, %.3f)",
      placement_alignment_physical[0], placement_alignment_physical[1],
      placement_alignment_physical[2], placement_alignment[0], placement_alignment[1],
      placement_alignment[2], alignment_segments, alignment_displacement,
      localization_bias_x, localization_bias_y, localization_bias_yaw);
    geometry_msgs::msg::PoseStamped previous_alignment_pose = dock_alignment_ground_truth;
    for (std::size_t segment = 1; segment <= alignment_segments; ++segment) {
      const double fraction = static_cast<double>(segment) /
        static_cast<double>(alignment_segments);
      const double previous_fraction = static_cast<double>(segment - 1) /
        static_cast<double>(alignment_segments);
      const double segment_start_x = dock_alignment_localized.pose.position.x +
        (placement_alignment[0] - dock_alignment_localized.pose.position.x) * previous_fraction;
      const double segment_start_y = dock_alignment_localized.pose.position.y +
        (placement_alignment[1] - dock_alignment_localized.pose.position.y) * previous_fraction;
      std::array<double, 3> segment_target{
        dock_alignment_localized.pose.position.x +
          (placement_alignment[0] - dock_alignment_localized.pose.position.x) * fraction,
        dock_alignment_localized.pose.position.y +
          (placement_alignment[1] - dock_alignment_localized.pose.position.y) * fraction,
        0.0};
      const double segment_heading = std::atan2(
        segment_target[1] - segment_start_y, segment_target[0] - segment_start_x);
      if (!std::isfinite(segment_heading))
        throw std::runtime_error("placement alignment segment bearing was non-finite");
      segment_target[2] = segment_heading;
      if (!node->navigate_to(segment_target, 120s))
        throw std::runtime_error("navigation to dispatch placement alignment segment failed");
      if (!product_attached || !node->native_attachment_state_is("attached"))
        throw std::runtime_error("attachment proof failed during placement alignment");
      geometry_msgs::msg::PoseStamped achieved_segment_pose;
      if (!node->latest_robot_pose(achieved_segment_pose))
        throw std::runtime_error("fresh alignment segment pose was unavailable");
      const double achieved_segment_displacement = std::hypot(
        achieved_segment_pose.pose.position.x - previous_alignment_pose.pose.position.x,
        achieved_segment_pose.pose.position.y - previous_alignment_pose.pose.position.y);
      if (!std::isfinite(achieved_segment_displacement) ||
        achieved_segment_displacement > kMaxPlacementAlignmentSegmentDisplacement)
      {
        throw std::runtime_error("achieved placement alignment segment exceeded 0.15 m");
      }
      previous_alignment_pose = achieved_segment_pose;
    }
    // Finish the bounded translation with an explicit terminal heading goal.
    // The preceding goals follow the bounded translation path; this terminal
    // goal is the only one that requests the dispatch heading. Re-sample the
    // localization bias immediately before this final goal because the dock
    // sample can drift during the bounded alignment.  A stale bias can leave
    // the ground-truth pose just outside the unchanged physical XY envelope.
    geometry_msgs::msg::PoseStamped final_heading_ground_truth;
    geometry_msgs::msg::PoseStamped final_heading_localized;
    if (!node->latest_robot_pose(final_heading_ground_truth) ||
      !node->latest_navigation_feedback_pose(final_heading_localized))
    {
      throw std::runtime_error("fresh final heading bias evidence was unavailable");
    }
    const double final_heading_ground_truth_yaw =
      yaw_from_pose(final_heading_ground_truth.pose);
    const double final_heading_localized_yaw =
      yaw_from_pose(final_heading_localized.pose);
    const double final_heading_bias_x =
      final_heading_ground_truth.pose.position.x - final_heading_localized.pose.position.x;
    const double final_heading_bias_y =
      final_heading_ground_truth.pose.position.y - final_heading_localized.pose.position.y;
    const double final_heading_bias_yaw =
      wrap_yaw(final_heading_ground_truth_yaw - final_heading_localized_yaw);
    const double final_heading_bias_delta = std::hypot(
      final_heading_bias_x - localization_bias_x,
      final_heading_bias_y - localization_bias_y);
    const double final_heading_bias_yaw_delta = std::abs(wrap_yaw(
      final_heading_bias_yaw - localization_bias_yaw));
    if (!std::isfinite(final_heading_bias_x) || !std::isfinite(final_heading_bias_y) ||
      !std::isfinite(final_heading_bias_yaw) ||
      !std::isfinite(final_heading_bias_delta) ||
      !std::isfinite(final_heading_bias_yaw_delta) ||
      final_heading_bias_delta > kMaxPlacementAlignmentPositionError ||
      final_heading_bias_yaw_delta > kMaxPlacementAlignmentYawError)
    {
      throw std::runtime_error("fresh final heading localization bias was out of bounds");
    }
    // Keep the unchanged 0.15 rad acceptance envelope, but command the final
    // heading 0.03 rad inside it so Nav2 cannot accept an already-near heading
    // without making the required rotation.
    const std::array<double, 3> final_heading_target{
      placement_alignment_physical[0] - final_heading_bias_x,
      placement_alignment_physical[1] - final_heading_bias_y,
      wrap_yaw(placement_alignment_physical[2] - final_heading_bias_yaw -
        kFinalHeadingGoalMargin)};
    if (!node->navigate_to(final_heading_target, 120s))
      throw std::runtime_error("navigation to final dispatch heading failed");
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed during final dispatch heading");
    geometry_msgs::msg::PoseStamped achieved_heading_pose;
    if (!node->latest_robot_pose(achieved_heading_pose))
      throw std::runtime_error("fresh final dispatch heading pose was unavailable");
    const double achieved_heading_displacement = std::hypot(
      achieved_heading_pose.pose.position.x - previous_alignment_pose.pose.position.x,
      achieved_heading_pose.pose.position.y - previous_alignment_pose.pose.position.y);
    if (!std::isfinite(achieved_heading_displacement) ||
      achieved_heading_displacement > kMaxPlacementAlignmentSegmentDisplacement)
    {
      throw std::runtime_error("final dispatch heading moved beyond the alignment segment bound");
    }
    previous_alignment_pose = achieved_heading_pose;
    if (!product_attached || !node->native_attachment_state_is("attached"))
      throw std::runtime_error("attachment proof failed after placement alignment");
    require_motion_permission();
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!node->latest_robot_pose(robot_pose) ||
      std::hypot(robot_pose.pose.position.x - previous_alignment_pose.pose.position.x,
        robot_pose.pose.position.y - previous_alignment_pose.pose.position.y) > 0.02)
      throw std::runtime_error("fresh robot pose was unavailable after placement alignment");
    const double alignment_yaw = std::atan2(
      2.0 * robot_pose.pose.orientation.w * robot_pose.pose.orientation.z,
      1.0 - 2.0 * robot_pose.pose.orientation.z * robot_pose.pose.orientation.z);
    const double alignment_position_error = std::hypot(
      robot_pose.pose.position.x - placement_alignment_physical[0],
      robot_pose.pose.position.y - placement_alignment_physical[1]);
    const double alignment_yaw_error = std::abs(std::remainder(
      alignment_yaw - placement_alignment_physical[2], 2.0 * std::acos(-1.0)));
    const double achieved_alignment_displacement = std::hypot(
      robot_pose.pose.position.x - dock_alignment_ground_truth.pose.position.x,
      robot_pose.pose.position.y - dock_alignment_ground_truth.pose.position.y);
    if (!std::isfinite(alignment_position_error) ||
      !std::isfinite(alignment_yaw_error) ||
      !std::isfinite(achieved_alignment_displacement) ||
      alignment_position_error > kMaxPlacementAlignmentPositionError ||
      alignment_yaw_error > kMaxPlacementAlignmentYawError ||
      achieved_alignment_displacement > kMaxPlacementAlignmentTotalDisplacement)
    {
      throw std::runtime_error(
              "achieved dispatch placement alignment exceeded the bounded pose envelope");
    }
    const auto aligned_attachment = node->measured_attachment_evidence(
      arm.getCurrentPose("gripper_tcp"));
    if (aligned_attachment.position_error_m > 0.030 ||
      aligned_attachment.orientation_error_rad > 0.15)
    {
      throw std::runtime_error("fresh product attachment evidence failed after placement alignment");
    }
    const auto release_product_map = std::array<double, 3>{
      selected_slot[0], selected_slot[1], selected_slot[2] + 0.020};
    const auto pre_place_product_map = std::array<double, 3>{
      selected_slot[0], selected_slot[1], selected_slot[2] + 0.200};
    const auto release_base = amr_manipulation::map_point_to_base(
      release_product_map, robot_pose);
    const auto pre_place_base = amr_manipulation::map_point_to_base(
      pre_place_product_map, robot_pose);
    auto dispatch_surface = amr_manipulation::box(
      "dispatch_surface", {product.size[0], product.size[1], 0.01},
      {amr_manipulation::map_point_to_base(
        {selected_slot[0], selected_slot[1], 0.005}, robot_pose)[0],
       amr_manipulation::map_point_to_base(
        {selected_slot[0], selected_slot[1], 0.005}, robot_pose)[1],
       amr_manipulation::map_point_to_base(
        {selected_slot[0], selected_slot[1], 0.005}, robot_pose)[2]});
    dispatch_surface.primitive_poses.front().orientation =
      amr_manipulation::inverse_yaw_quaternion(robot_pose);
    if (!scene.applyCollisionObject(dispatch_surface))
      throw std::runtime_error("dispatch surface collision object was rejected");

    geometry_msgs::msg::Pose pre_place = grasp;
    pre_place.position.x = pre_place_base[0];
    pre_place.position.y = pre_place_base[1];
    pre_place.position.z = pre_place_base[2] + 0.080;
    const double pre_place_radial_yaw = std::atan2(pre_place_base[1], pre_place_base[0]);
    if (!std::isfinite(pre_place_radial_yaw))
      throw std::runtime_error("placement radial direction was invalid");
    const double release_radius = std::hypot(release_base[0], release_base[1]);
    if (!std::isfinite(release_radius) || release_radius > kMaxPlacementReleaseRadius)
      throw std::runtime_error("placement target was outside the deterministic IK envelope");
    // The held product is represented in base_footprint.  Invert the fresh
    // base/map yaw so its top-down yaw is map-axis aligned, choosing the
    // equivalent pi branch closest to the map x axis.  Keep the radial yaw
    // only as the deterministic reach/IK seed below.
    const double map_aligned_product_yaw = wrap_yaw(-alignment_yaw);
    const double map_aligned_product_yaw_pi = wrap_yaw(
      map_aligned_product_yaw + std::acos(-1.0));
    const double pre_place_map_yaw =
      std::abs(map_aligned_product_yaw) <= std::abs(map_aligned_product_yaw_pi) ?
      map_aligned_product_yaw : map_aligned_product_yaw_pi;
    if (!std::isfinite(alignment_yaw) || !std::isfinite(map_aligned_product_yaw) ||
      !std::isfinite(pre_place_map_yaw))
      throw std::runtime_error("map-aligned placement yaw was invalid");
    pre_place.orientation = amr_manipulation::top_down_radial_quaternion(
      pre_place_map_yaw);
    geometry_msgs::msg::Pose release = pre_place;
    release.position.x = release_base[0];
    release.position.y = release_base[1];
    release.position.z = release_base[2] + 0.080;

    // KDL's reachable placement branch is narrow.  Probe the release endpoint
    // with an explicit deterministic seed, then continue that branch upward
    // before asking OMPL to move from loaded stow; never fall back to
    // approximate IK or the stow seed.
    const std::vector<double> placement_release_seed{
      -pre_place_radial_yaw, 0.546225552, 0.335934775, 0.0, -0.882160326, 0.0};
    auto placement_ik_state = arm.getCurrentState(3.0);
    if (!placement_ik_state)
      throw std::runtime_error("fresh MoveIt state was unavailable for placement IK preflight");
    const auto * manipulator_group = placement_ik_state->getJointModelGroup("manipulator");
    const std::vector<std::string> expected_placement_joint_names{
      "arm_joint_1", "arm_joint_2", "arm_joint_3",
      "arm_joint_4", "arm_joint_5", "arm_joint_6"};
    if (!manipulator_group ||
      manipulator_group->getVariableNames() != expected_placement_joint_names ||
      manipulator_group->getVariableCount() != placement_release_seed.size())
    {
      throw std::runtime_error("placement IK manipulator joint order was invalid");
    }
    const auto finite_joint_values = [](const std::vector<double> & values) {
        return !values.empty() && std::all_of(values.begin(), values.end(),
          [](double value) { return std::isfinite(value); });
      };
    if (!finite_joint_values(placement_release_seed))
    {
      throw std::runtime_error("placement IK seeds were invalid");
    }
    placement_ik_state->setJointGroupPositions(manipulator_group, placement_release_seed);
    placement_ik_state->update();
    if (!placement_ik_state->setFromIK(
        manipulator_group, release, "gripper_tcp", 0.5) ||
      !placement_ik_state->satisfiesBounds(manipulator_group))
    {
      throw std::runtime_error("exact seeded release IK preflight failed");
    }
    std::vector<double> release_ik_solution;
    placement_ik_state->copyJointGroupPositions(manipulator_group, release_ik_solution);
    if (!finite_joint_values(release_ik_solution))
      throw std::runtime_error("release IK preflight returned invalid joints");

    // Continue deterministically from the release branch in 5 mm increments;
    // this keeps the pre-place solution on the same collision-safe branch even
    // when the accepted navigation terminal pose is a few centimetres away
    // from the nominal alignment target.
    const double vertical_delta = pre_place.position.z - release.position.z;
    const auto continuation_steps = static_cast<std::size_t>(
      std::ceil(std::abs(vertical_delta) / 0.005));
    if (continuation_steps == 0 || continuation_steps > 100)
      throw std::runtime_error("placement IK continuation geometry was invalid");
    std::vector<double> continuation_solution = release_ik_solution;
    for (std::size_t step = 1; step <= continuation_steps; ++step) {
      auto continuation_pose = release;
      continuation_pose.position.z += vertical_delta *
        static_cast<double>(step) / static_cast<double>(continuation_steps);
      placement_ik_state->setJointGroupPositions(manipulator_group, continuation_solution);
      placement_ik_state->update();
      if (!placement_ik_state->setFromIK(
          manipulator_group, continuation_pose, "gripper_tcp", 0.5) ||
        !placement_ik_state->satisfiesBounds(manipulator_group))
      {
        throw std::runtime_error("deterministic placement IK continuation failed");
      }
      placement_ik_state->copyJointGroupPositions(manipulator_group, continuation_solution);
      if (!finite_joint_values(continuation_solution))
        throw std::runtime_error("placement IK continuation returned invalid joints");
    }
    std::vector<double> pre_place_ik_solution;
    pre_place_ik_solution = continuation_solution;
    if (!finite_joint_values(pre_place_ik_solution))
      throw std::runtime_error("pre-place IK preflight returned invalid joints");

    arm.setStartStateToCurrentState();
    if (!arm.setJointValueTarget(pre_place_ik_solution))
      throw std::runtime_error("exact pre-place joint target was rejected");
    MoveGroupInterface::Plan pre_place_plan;
    if (arm.plan(pre_place_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("pre-place planning failed");
    require_motion_permission();
    if (arm.execute(pre_place_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("pre-place execution failed");

    std::vector<geometry_msgs::msg::Pose> lower_waypoints{release};
    moveit_msgs::msg::RobotTrajectory lower_trajectory;
    if (arm.computeCartesianPath(
        lower_waypoints, 0.005, 0.0, lower_trajectory, true) < 0.99)
      throw std::runtime_error("Cartesian placement lower was incomplete");
    MoveGroupInterface::Plan lower_plan;
    lower_plan.trajectory_ = lower_trajectory;
    require_motion_permission();
    if (arm.execute(lower_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Cartesian placement lower execution failed");
    if (node->selected_slot_position_error() > 0.030)
      throw std::runtime_error("selected dispatch slot center exceeded 30 mm");
    const auto detach_decision = attachment_gate.evaluate_detach(
      product.id, node->nearest_slot_position_error());
    if (node->selected_slot_position_error() > 0.030 ||
      detach_decision != amr_manipulation::AttachmentDecision::ACCEPT)
    {
      throw std::runtime_error("dispatch detachment pose was out of tolerance");
    }
    if (!node->request_and_confirm_operational_detachment(3s))
      throw std::runtime_error("native dispatch detachment confirmation timed out");
    if (!attachment_gate.confirm_detached(product.id))
      throw std::runtime_error("fresh native detachment confirmation was not accepted");
    product_attached = false;
    node->set_status(amr_interfaces::msg::ManipulatorStatus::MOVING,
      false, false, "Dispatch detachment confirmed; opening gripper");
    require_motion_permission();
    if (!amr_manipulation::command_gripper(node, 0.035))
      throw std::runtime_error("gripper opening after detachment failed");
    if (!node->gripper_positions_above(0.034, 3s))
      throw std::runtime_error("fresh bilateral open gripper positions were not proven");

    moveit_msgs::msg::AttachedCollisionObject remove_attached;
    remove_attached.link_name = "gripper_left_finger_link";
    remove_attached.object.id = "held_product";
    remove_attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene.applyAttachedCollisionObject(remove_attached))
      throw std::runtime_error("held product scene removal was rejected");
    geometry_msgs::msg::PoseStamped measured_product;
    if (!node->latest_product_pose(measured_product) ||
      !node->latest_robot_pose(robot_pose))
    {
      throw std::runtime_error("fresh post-detach product scene evidence was unavailable");
    }
    if (!set_held_product_finger_collision(true))
      throw std::runtime_error("temporary held-product finger collision allowance was rejected");
    try {
      std::vector<geometry_msgs::msg::Pose> retreat_waypoints{pre_place};
      moveit_msgs::msg::RobotTrajectory retreat_trajectory;
      if (arm.computeCartesianPath(
          retreat_waypoints, 0.005, 0.0, retreat_trajectory, true) < 0.99)
        throw std::runtime_error("Cartesian placement retreat was incomplete");
      MoveGroupInterface::Plan retreat_plan;
      retreat_plan.trajectory_ = retreat_trajectory;
      require_motion_permission();
      if (arm.execute(retreat_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("Cartesian placement retreat execution failed");
    } catch (...) {
      if (!set_held_product_finger_collision(false))
        RCLCPP_ERROR(node->get_logger(),
          "Failed to restore held-product finger collision checking");
      throw;
    }
    if (!set_held_product_finger_collision(false))
      throw std::runtime_error("held-product finger collision checking was not restored");

    auto post_retreat_state = arm.getCurrentState(3.0);
    if (!post_retreat_state)
      throw std::runtime_error("fresh post-retreat MoveIt state was unavailable");
    if (!validity_client->wait_for_service(3s))
      throw std::runtime_error("/check_state_validity service was unavailable after retreat");
    auto post_retreat_validity_request =
      std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    moveit::core::robotStateToRobotStateMsg(
      *post_retreat_state, post_retreat_validity_request->robot_state);
    post_retreat_validity_request->group_name = "manipulator";
    auto post_retreat_validity_future =
      validity_client->async_send_request(post_retreat_validity_request);
    if (post_retreat_validity_future.wait_for(3s) != std::future_status::ready)
      throw std::runtime_error("/check_state_validity request timed out after retreat");
    const auto post_retreat_validity_response = post_retreat_validity_future.get();
    if (!post_retreat_validity_response)
      throw std::runtime_error("/check_state_validity returned no response after retreat");
    for (const auto & contact : post_retreat_validity_response->contacts) {
      RCLCPP_WARN(
        node->get_logger(), "Post-retreat MoveIt state contact: %s <-> %s",
        contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
    }
    if (!post_retreat_validity_response->valid)
      throw std::runtime_error("post-detach retreat state is invalid");

    if (!arm.setJointValueTarget(stow))
      throw std::runtime_error("empty stow target was rejected");
    MoveGroupInterface::Plan empty_stow_plan;
    if (arm.plan(empty_stow_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("empty stow planning failed");
    require_motion_permission();
    if (arm.execute(empty_stow_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("empty stow execution failed");
    const auto empty_current = arm.getCurrentJointValues();
    if (empty_current.size() != stow.size())
      throw std::runtime_error("empty stow joint state is incomplete");
    for (std::size_t i = 0; i < stow.size(); ++i) {
      if (std::abs(empty_current[i] - stow[i]) > 0.01)
        throw std::runtime_error("empty stow tolerance was not achieved");
    }
    node->set_status(amr_interfaces::msg::ManipulatorStatus::STOWED_EMPTY,
      true, false, "Gate 6 " + std::to_string(product.mass_kg) +
      " kg grasp, transport, placement, and empty stow passed");
    RCLCPP_INFO(node->get_logger(), "GATE 6 %.1f KG COMPLETE 1 KG PASS", product.mass_kg);
    passed = true;
  } catch (const std::exception & error) {
    node->set_status(amr_interfaces::msg::ManipulatorStatus::FAULT,
      false, product_attached, error.what());
    RCLCPP_ERROR(node->get_logger(), "GATE 6 %.1f KG: FAIL: %s", product.mass_kg, error.what());
  }
  std::this_thread::sleep_for(500ms);
  rclcpp::shutdown();
  spin_thread.join();
  return passed ? 0 : 1;
}
