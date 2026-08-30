#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_control {

using namespace std::chrono_literals;
using SteadyTime = std::chrono::steady_clock::time_point;
using BackUp = nav2_msgs::action::BackUp;
using GoalHandleBackUp = rclcpp_action::ServerGoalHandle<BackUp>;

// This lifecycle node is the sole project-owned velocity publisher.  Normal
// Nav2 requests are bounded here; the bounded pickup-dock retreat is also
// arbitrated here so no second command publisher can bypass the safety gate.
class CommandArbitrationNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit CommandArbitrationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("command_arbitration_node", options),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    source_timeout_ = std::chrono::milliseconds(
      declare_parameter("source_timeout_ms", 200));
    require_manipulator_stowed_ =
      declare_parameter("require_manipulator_stowed", false);
    manipulator_status_timeout_ = std::chrono::milliseconds(
      declare_parameter("manipulator_status_timeout_ms", 200));
    output_frequency_ = declare_parameter("output_frequency", 20.0);
    max_linear_velocity_ = declare_parameter("max_linear_velocity", 0.5);
    max_angular_velocity_ = declare_parameter("max_angular_velocity", 0.4);
    max_linear_acceleration_ =
      declare_parameter("max_linear_acceleration", 0.5);
    max_angular_acceleration_ =
      declare_parameter("max_angular_acceleration", 0.4);
    egress_max_distance_ = declare_parameter("egress_max_distance_m", 0.50);
    egress_max_speed_ = declare_parameter("egress_max_speed_mps", 0.10);
    egress_time_limit_s_ = declare_parameter("egress_time_limit_s", 60.0);
    egress_status_timeout_ = std::chrono::milliseconds(
      declare_parameter("egress_status_timeout_ms", 200));
    egress_odometry_timeout_ = std::chrono::milliseconds(
      declare_parameter("egress_odometry_timeout_ms", 200));
    egress_scan_timeout_ = std::chrono::milliseconds(
      declare_parameter("egress_scan_timeout_ms", 1000));
    egress_drift_tolerance_ =
      declare_parameter("egress_drift_tolerance_m", 0.05);
    egress_yaw_tolerance_ =
      declare_parameter("egress_yaw_tolerance_rad", 0.05);
    egress_clearance_ = declare_parameter("egress_clearance_m", 0.05);
    egress_footprint_text_ = declare_parameter<std::string>(
      "egress_footprint",
      "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    if (source_timeout_.count() <= 0 ||
      manipulator_status_timeout_.count() <= 0 ||
      !std::isfinite(output_frequency_) || output_frequency_ <= 0.0 ||
      !std::isfinite(max_linear_velocity_) || max_linear_velocity_ <= 0.0 ||
      !std::isfinite(max_angular_velocity_) || max_angular_velocity_ <= 0.0 ||
      !std::isfinite(max_linear_acceleration_) || max_linear_acceleration_ <= 0.0 ||
      !std::isfinite(max_angular_acceleration_) || max_angular_acceleration_ <= 0.0 ||
      !std::isfinite(egress_max_distance_) || egress_max_distance_ <= 0.0 ||
      !std::isfinite(egress_max_speed_) || egress_max_speed_ <= 0.0 ||
      !std::isfinite(egress_time_limit_s_) || egress_time_limit_s_ <= 0.0 ||
      egress_status_timeout_.count() <= 0 ||
      egress_odometry_timeout_.count() <= 0 ||
      egress_scan_timeout_.count() <= 0 ||
      !std::isfinite(egress_drift_tolerance_) || egress_drift_tolerance_ <= 0.0 ||
      !std::isfinite(egress_yaw_tolerance_) || egress_yaw_tolerance_ <= 0.0 ||
      !std::isfinite(egress_clearance_) || egress_clearance_ <= 0.0 ||
      !parse_footprint(egress_footprint_text_, egress_footprint_))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Timeout, frequency, velocity, acceleration, egress, or footprint parameters are invalid");
      return CallbackReturn::FAILURE;
    }

    request_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/amr/control/cmd_vel", amr_interfaces::qos::command());
    source_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/amr/mpc/cmd_vel", amr_interfaces::qos::nav2_command_input(),
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        receive_source(*message);
      });
    // Keep this subscription present even when the normal factory interlock
    // is disabled: the BackUp action always requires loaded-stow evidence.
    manipulator_status_sub_ =
      create_subscription<amr_interfaces::msg::ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority(),
      [this](amr_interfaces::msg::ManipulatorStatus::SharedPtr message) {
        receive_manipulator_status(*message);
      });
    base_status_sub_ = create_subscription<amr_interfaces::msg::BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic(),
      [this](amr_interfaces::msg::BaseStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        base_status_ = *message;
        have_base_status_ = true;
        last_base_status_ = std::chrono::steady_clock::now();
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/amr/localization/odometry", amr_interfaces::qos::sensor(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        odometry_ = *message;
        have_odometry_ = true;
        last_odometry_ = std::chrono::steady_clock::now();
      });
    rear_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/amr/sensors/rear_lidar/scan", amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        rear_scan_ = *message;
        have_rear_scan_ = true;
        last_rear_scan_ = std::chrono::steady_clock::now();
      });
    egress_action_server_ = rclcpp_action::create_server<BackUp>(
      this, "/amr/control/dock_egress",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const BackUp::Goal> goal) {
        return handle_egress_goal(goal);
      },
      [this](const std::shared_ptr<GoalHandleBackUp> goal_handle) {
        return handle_egress_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleBackUp> goal_handle) {
        handle_egress_accepted(goal_handle);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    request_pub_->on_activate();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      have_source_ = false;
      source_valid_ = false;
      source_active_ = false;
      have_manipulator_status_ = false;
      manipulator_status_valid_ = false;
      have_base_status_ = false;
      have_odometry_ = false;
      have_rear_scan_ = false;
      manipulator_boot_id_ = 0;
      manipulator_sequence_ = 0;
      egress_running_ = false;
      lifecycle_deactivating_ = false;
      last_linear_ = 0.0;
      last_angular_ = 0.0;
      last_output_ = std::chrono::steady_clock::now();
      suppress_source_until_ = SteadyTime{};
    }
    const auto period = std::chrono::duration<double>(1.0 / output_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    bool must_stop = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      lifecycle_deactivating_ = true;
      must_stop = egress_running_ || source_active_;
      source_active_ = false;
      have_source_ = false;
      source_valid_ = false;
      last_linear_ = 0.0;
      last_angular_ = 0.0;
    }
    timer_.reset();
    if (must_stop && request_pub_->is_activated()) {
      publish(0.0, 0.0);
    }
    request_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

 private:
  struct Point2 {
    double x;
    double y;
  };

  static bool valid_planar(const geometry_msgs::msg::Twist & command) {
    const auto finite = [](double value) { return std::isfinite(value); };
    return finite(command.linear.x) && finite(command.linear.y) &&
      finite(command.linear.z) && finite(command.angular.x) &&
      finite(command.angular.y) && finite(command.angular.z) &&
      command.linear.y == 0.0 && command.linear.z == 0.0 &&
      command.angular.x == 0.0 && command.angular.y == 0.0;
  }

  static double slew(double target, double current, double limit, double seconds) {
    const double maximum_change = limit * std::max(0.0, seconds);
    return current + std::clamp(target - current, -maximum_change, maximum_change);
  }

  static double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static bool valid_manipulator_semantics(
    const amr_interfaces::msg::ManipulatorStatus & status)
  {
    using Status = amr_interfaces::msg::ManipulatorStatus;
    if (!status.valid || !status.base_motion_allowed) return false;
    if (status.state == Status::STOWED_EMPTY) {
      return !status.product_attached && status.product_id.empty();
    }
    if (status.state == Status::STOWED_LOADED) {
      return status.product_attached && !status.product_id.empty();
    }
    return false;
  }

  static bool parse_footprint(const std::string & text, std::vector<Point2> & result) {
    result.clear();
    const char * cursor = text.c_str();
    while (*cursor != '\0') {
      char * end = nullptr;
      const double value = std::strtod(cursor, &end);
      if (end != cursor) {
        if (!std::isfinite(value)) return false;
        result.push_back({value, 0.0});
        cursor = end;
      } else {
        ++cursor;
      }
    }
    if (result.size() != 8U) return false;
    for (std::size_t index = 0; index < result.size(); index += 2) {
      result[index / 2].y = result[index + 1].x;
      result[index / 2].x = result[index].x;
    }
    result.resize(4U);
    const auto area = [&result]() {
        double sum = 0.0;
        for (std::size_t i = 0; i < result.size(); ++i) {
          const auto & lhs = result[i];
          const auto & rhs = result[(i + 1) % result.size()];
          sum += lhs.x * rhs.y - rhs.x * lhs.y;
        }
        return std::abs(sum) * 0.5;
      }();
    return area > 0.0;
  }

  void receive_source(const geometry_msgs::msg::Twist & message) {
    const auto steady_now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Samples received while a dock egress is active are deliberately
    // discarded.  A short post-egress suppression window prevents a queued
    // pre-egress Nav2 sample from being replayed after the action ends.
    if (egress_running_ || steady_now < suppress_source_until_) return;
    source_command_ = message;
    source_valid_ = valid_planar(message);
    have_source_ = true;
    last_source_ = steady_now;
  }

  void receive_manipulator_status(
    const amr_interfaces::msg::ManipulatorStatus & status)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool identity_valid = status.source_boot_id != 0U && status.sequence != 0U;
    bool sequence_valid = identity_valid;
    if (identity_valid && have_manipulator_status_ &&
      status.source_boot_id == manipulator_boot_id_)
    {
      sequence_valid = status.sequence > manipulator_sequence_;
    }
    have_manipulator_status_ = true;
    last_manipulator_status_ = steady_now;
    last_manipulator_message_ = status;
    manipulator_status_valid_ = sequence_valid && valid_manipulator_semantics(status);
    if (sequence_valid) {
      manipulator_boot_id_ = status.source_boot_id;
      manipulator_sequence_ = status.sequence;
    }
  }

  bool normal_motion_allowed_locked(const SteadyTime now) const {
    const bool fresh = have_source_ && source_valid_ &&
      now - last_source_ <= source_timeout_;
    const bool manipulator_allows_motion = !require_manipulator_stowed_ ||
      (have_manipulator_status_ && manipulator_status_valid_ &&
      now - last_manipulator_status_ <= manipulator_status_timeout_);
    return fresh && manipulator_allows_motion;
  }

  void tick() {
    const auto steady_now = std::chrono::steady_clock::now();
    double linear = 0.0;
    double angular = 0.0;
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (egress_running_) {
        const double elapsed =
          std::chrono::duration<double>(steady_now - last_output_).count();
        linear = slew(
          -egress_speed_, last_linear_, max_linear_acceleration_, elapsed);
        angular = slew(0.0, last_angular_, max_angular_acceleration_, elapsed);
        last_linear_ = linear;
        last_angular_ = angular;
        last_output_ = steady_now;
        should_publish = true;
      } else if (normal_motion_allowed_locked(steady_now)) {
        const double elapsed =
          std::chrono::duration<double>(steady_now - last_output_).count();
        const double linear_target = std::clamp(
          source_command_.linear.x, -max_linear_velocity_, max_linear_velocity_);
        const double angular_target = std::clamp(
          source_command_.angular.z, -max_angular_velocity_, max_angular_velocity_);
        linear = slew(linear_target, last_linear_, max_linear_acceleration_, elapsed);
        angular = slew(angular_target, last_angular_, max_angular_acceleration_, elapsed);
        last_linear_ = linear;
        last_angular_ = angular;
        source_active_ = true;
        last_output_ = steady_now;
        should_publish = true;
      } else {
        if (source_active_) {
          source_active_ = false;
          last_linear_ = 0.0;
          last_angular_ = 0.0;
          should_publish = true;
        }
        last_output_ = steady_now;
      }
    }
    if (should_publish) publish(linear, angular);
  }

  bool valid_egress_goal(const BackUp::Goal & goal, std::string & reason) const {
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!finite(goal.target.x) || !finite(goal.target.y) || !finite(goal.target.z) ||
      !finite(goal.speed) || goal.target.x <= 0.0 || goal.target.x > egress_max_distance_ ||
      goal.target.y != 0.0 || goal.target.z != 0.0 || goal.speed <= 0.0 ||
      static_cast<double>(goal.speed) > egress_max_speed_ + 1e-6)
    {
      reason = "target must be positive reverse X and within configured distance/speed limits";
      return false;
    }
    if (goal.time_allowance.sec < 0 || goal.time_allowance.nanosec >= 1000000000U) {
      reason = "time allowance is malformed";
      return false;
    }
    const double allowance = static_cast<double>(goal.time_allowance.sec) +
      static_cast<double>(goal.time_allowance.nanosec) * 1e-9;
    if (!finite(allowance) || allowance <= 0.0 || allowance > egress_time_limit_s_) {
      reason = "time allowance is outside the configured wall-clock limit";
      return false;
    }
    return true;
  }

  bool base_ready_locked(const SteadyTime now) const {
    return have_base_status_ && now - last_base_status_ <= egress_status_timeout_ &&
      base_status_.valid && base_status_.source_boot_id != 0U &&
      base_status_.sequence != 0U &&
      base_status_.state == amr_interfaces::msg::BaseStatus::READY &&
      base_status_.reason == amr_interfaces::msg::BaseStatus::REASON_READY;
  }

  static bool finite_pose(const geometry_msgs::msg::Pose & pose) {
    const auto finite = [](double value) { return std::isfinite(value); };
    return finite(pose.position.x) && finite(pose.position.y) &&
      finite(pose.position.z) && finite(pose.orientation.x) &&
      finite(pose.orientation.y) && finite(pose.orientation.z) &&
      finite(pose.orientation.w);
  }

  bool odometry_fresh_locked(const SteadyTime now) const {
    if (!have_odometry_ || now - last_odometry_ > egress_odometry_timeout_ ||
      !finite_pose(odometry_.pose.pose)) return false;
    const auto & twist = odometry_.twist.twist;
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
      std::isfinite(twist.angular.z);
  }

  bool loaded_stow_fresh_locked(const SteadyTime now) const {
    return have_manipulator_status_ &&
      now - last_manipulator_status_ <= egress_status_timeout_ &&
      manipulator_status_valid_ &&
      last_manipulator_message_.state ==
      amr_interfaces::msg::ManipulatorStatus::STOWED_LOADED &&
      last_manipulator_message_.product_attached &&
      !last_manipulator_message_.product_id.empty();
  }

  bool no_active_nonzero_nav_locked(const SteadyTime now) const {
    const bool fresh = have_source_ && source_valid_ &&
      now - last_source_ <= source_timeout_;
    if (!fresh) return true;
    return std::abs(source_command_.linear.x) <= 1e-9 &&
      std::abs(source_command_.angular.z) <= 1e-9;
  }

  bool ray_intersects_rectangle(
    const Point2 & origin, const Point2 & direction,
    double x_min, double x_max, double y_min, double y_max,
    double & exit_distance) const
  {
    double near_distance = 0.0;
    double far_distance = std::numeric_limits<double>::infinity();
    const auto slab = [&near_distance, &far_distance](
      double origin_axis, double direction_axis, double min_axis, double max_axis) {
        if (std::abs(direction_axis) < 1e-9) {
          return origin_axis >= min_axis && origin_axis <= max_axis;
        }
        double near_value = (min_axis - origin_axis) / direction_axis;
        double far_value = (max_axis - origin_axis) / direction_axis;
        if (near_value > far_value) std::swap(near_value, far_value);
        near_distance = std::max(near_distance, near_value);
        far_distance = std::min(far_distance, far_value);
        return near_distance <= far_distance;
      };
    if (!slab(origin.x, direction.x, x_min, x_max) ||
      !slab(origin.y, direction.y, y_min, y_max) || far_distance < 0.0)
    {
      return false;
    }
    exit_distance = far_distance;
    return std::isfinite(exit_distance) && exit_distance >= 0.0;
  }

  bool scan_corridor_clear(double travel_distance, std::string & reason) {
    sensor_msgs::msg::LaserScan scan;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (!have_rear_scan_ || now - last_rear_scan_ > egress_scan_timeout_) {
        reason = "rear LiDAR evidence is stale";
        return false;
      }
      scan = rear_scan_;
    }
    if (scan.header.frame_id.empty() || scan.ranges.size() < 180U ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0 ||
      scan.angle_max - scan.angle_min < 4.0 ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0.0 || scan.range_max <= scan.range_min)
    {
      reason = "rear LiDAR coverage or metadata is insufficient";
      return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (scan.header.frame_id == "base_footprint") {
      transform.transform.rotation.w = 1.0;
    } else {
      try {
        const auto nanoseconds =
          static_cast<int64_t>(scan.header.stamp.sec) * 1000000000LL +
          static_cast<int64_t>(scan.header.stamp.nanosec);
        const tf2::TimePoint stamp{std::chrono::nanoseconds(nanoseconds)};
        transform = tf_buffer_.lookupTransform(
          "base_footprint", scan.header.frame_id, stamp, tf2::durationFromSec(0.05));
      } catch (const std::exception & error) {
        reason = std::string("rear LiDAR TF unavailable: ") + error.what();
        return false;
      }
    }
    tf2::Quaternion rotation;
    tf2::fromMsg(transform.transform.rotation, rotation);
    const tf2::Vector3 translation(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    if (!std::isfinite(translation.x()) || !std::isfinite(translation.y()) ||
      !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
      !std::isfinite(rotation.z()) || !std::isfinite(rotation.w()))
    {
      reason = "rear LiDAR TF is malformed";
      return false;
    }

    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const auto & point : egress_footprint_) {
      x_min = std::min(x_min, point.x);
      x_max = std::max(x_max, point.x);
      y_min = std::min(y_min, point.y);
      y_max = std::max(y_max, point.y);
    }
    x_min -= travel_distance + egress_clearance_;
    x_max += egress_clearance_;
    y_min -= egress_clearance_;
    y_max += egress_clearance_;

    const Point2 origin{translation.x(), translation.y()};
    std::size_t covered = 0U;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      double range = scan.ranges[index];
      if (std::isinf(range) && range > 0.0) range = scan.range_max;
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        reason = "rear LiDAR contains malformed or out-of-range samples";
        return false;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const tf2::Vector3 sensor_direction(std::cos(angle), std::sin(angle), 0.0);
      const tf2::Vector3 base_direction = tf2::quatRotate(rotation, sensor_direction);
      const double norm = std::hypot(base_direction.x(), base_direction.y());
      if (!std::isfinite(norm) || norm < 1e-9) {
        reason = "rear LiDAR ray transform is malformed";
        return false;
      }
      const Point2 direction{base_direction.x() / norm, base_direction.y() / norm};
      if (direction.x >= -1e-9) continue;
      double exit_distance = 0.0;
      if (!ray_intersects_rectangle(
          origin, direction, x_min, x_max, y_min, y_max, exit_distance))
      {
        continue;
      }
      ++covered;
      if (range + 1e-3 < exit_distance) {
        reason = "rear LiDAR detected an obstruction in the swept corridor";
        return false;
      }
    }
    if (covered < scan.ranges.size() / 6U || covered < 20U) {
      reason = "rear LiDAR does not cover the complete swept corridor";
      return false;
    }
    return true;
  }

  rclcpp_action::GoalResponse handle_egress_goal(
    const std::shared_ptr<const BackUp::Goal> & goal)
  {
    std::string reason;
    if (!goal || !valid_egress_goal(*goal, reason)) {
      RCLCPP_WARN(get_logger(), "Dock egress goal rejected: %s", reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(get_logger(), "Dock egress goal rejected: lifecycle node is not active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (egress_running_) {
        RCLCPP_WARN(get_logger(), "Dock egress goal rejected: another egress is running");
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (lifecycle_deactivating_ || !loaded_stow_fresh_locked(now) ||
        !base_ready_locked(now) || !odometry_fresh_locked(now) ||
        !no_active_nonzero_nav_locked(now))
      {
        RCLCPP_WARN(
          get_logger(),
          "Dock egress goal rejected: fresh loaded-stow, READY base, filtered odometry, "
          "and idle Nav2 evidence are required");
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (!have_rear_scan_ || now - last_rear_scan_ > egress_scan_timeout_) {
        RCLCPP_WARN(get_logger(), "Dock egress goal rejected: rear LiDAR evidence is stale");
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    // Reserve the single action slot before returning ACCEPT so a concurrent
    // goal cannot pass the same evidence check in another DDS callback.
    std::string scan_reason;
    if (!scan_corridor_clear(goal->target.x, scan_reason)) {
      RCLCPP_WARN(get_logger(), "Dock egress goal rejected: %s", scan_reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (egress_running_) {
        RCLCPP_WARN(get_logger(), "Dock egress goal rejected: another egress started concurrently");
        return rclcpp_action::GoalResponse::REJECT;
      }
      egress_running_ = true;
      egress_distance_ = goal->target.x;
      egress_speed_ = goal->speed;
      egress_started_ = now;
      egress_allowance_s_ = static_cast<double>(goal->time_allowance.sec) +
        static_cast<double>(goal->time_allowance.nanosec) * 1e-9;
      egress_start_odom_ = odometry_.pose.pose;
      egress_start_yaw_ = yaw_from_pose(egress_start_odom_);
      source_active_ = false;
      have_source_ = false;
      source_valid_ = false;
      last_linear_ = 0.0;
      last_angular_ = 0.0;
      last_output_ = now;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_egress_cancel(
    const std::shared_ptr<GoalHandleBackUp> &) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_egress_accepted(const std::shared_ptr<GoalHandleBackUp> & goal_handle) {
    std::thread([this, goal_handle]() { execute_egress(goal_handle); }).detach();
  }

  static double yaw_from_pose(const geometry_msgs::msg::Pose & pose) {
    return std::atan2(
      2.0 * pose.orientation.w * pose.orientation.z,
      1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
  }

  double measured_travel_locked() const {
    const double dx = odometry_.pose.pose.position.x - egress_start_odom_.position.x;
    const double dy = odometry_.pose.pose.position.y - egress_start_odom_.position.y;
    return -(dx * std::cos(egress_start_yaw_) + dy * std::sin(egress_start_yaw_));
  }

  void finish_egress(
    const std::shared_ptr<GoalHandleBackUp> & goal_handle,
    rclcpp_action::ResultCode terminal_code, const std::string & reason)
  {
    double travel = 0.0;
    double elapsed = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      travel = have_odometry_ ? measured_travel_locked() : 0.0;
      elapsed = egress_started_ == SteadyTime{} ? 0.0 :
        std::chrono::duration<double>(std::chrono::steady_clock::now() - egress_started_).count();
      egress_running_ = false;
      source_active_ = false;
      have_source_ = false;
      source_valid_ = false;
      suppress_source_until_ = std::chrono::steady_clock::now() + source_timeout_;
      last_linear_ = 0.0;
      last_angular_ = 0.0;
    }
    if (request_pub_ && request_pub_->is_activated()) publish(0.0, 0.0);
    RCLCPP_INFO(
      get_logger(), "Dock egress terminal=%s measured_travel=%.3f m elapsed_wall=%.3f s",
      reason.c_str(), travel, elapsed);
    auto result = std::make_shared<BackUp::Result>();
    const auto elapsed_ns = static_cast<int64_t>(std::max(0.0, elapsed) * 1e9);
    result->total_elapsed_time.sec = static_cast<int32_t>(elapsed_ns / 1000000000LL);
    result->total_elapsed_time.nanosec = static_cast<uint32_t>(elapsed_ns % 1000000000LL);
    if (!goal_handle) return;
    if (terminal_code == rclcpp_action::ResultCode::CANCELED) {
      goal_handle->canceled(result);
    } else if (terminal_code == rclcpp_action::ResultCode::SUCCEEDED) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void execute_egress(const std::shared_ptr<GoalHandleBackUp> & goal_handle) {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(egress_allowance_s_));
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        finish_egress(goal_handle, rclcpp_action::ResultCode::CANCELED, "CANCELED");
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        finish_egress(goal_handle, rclcpp_action::ResultCode::ABORTED, "TIMEOUT");
        return;
      }
      std::string failure_reason;
      double travel = 0.0;
      double lateral = 0.0;
      double yaw_error = 0.0;
      bool evidence_ok = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (lifecycle_deactivating_) {
          failure_reason = "LIFECYCLE_DEACTIVATED";
        } else if (!loaded_stow_fresh_locked(now)) {
          failure_reason = "LOADED_STOW_INTERLOCK_LOST";
        } else if (!base_ready_locked(now)) {
          failure_reason = "BASE_READY_EVIDENCE_STALE_OR_INVALID";
        } else if (!odometry_fresh_locked(now)) {
          failure_reason = "FILTERED_ODOMETRY_STALE_OR_INVALID";
        } else {
          travel = measured_travel_locked();
          const double dx = odometry_.pose.pose.position.x - egress_start_odom_.position.x;
          const double dy = odometry_.pose.pose.position.y - egress_start_odom_.position.y;
          lateral = -dx * std::sin(egress_start_yaw_) + dy * std::cos(egress_start_yaw_);
          yaw_error = normalize_angle(yaw_from_pose(odometry_.pose.pose) - egress_start_yaw_);
          if (travel < -egress_drift_tolerance_) {
            failure_reason = "NON_REVERSE_ODOMETRY_DRIFT";
          } else if (std::abs(lateral) > egress_drift_tolerance_) {
            failure_reason = "LATERAL_ODOMETRY_DRIFT";
          } else if (std::abs(yaw_error) > egress_yaw_tolerance_) {
            failure_reason = "YAW_ODOMETRY_DRIFT";
          } else {
            evidence_ok = true;
          }
        }
      }
      if (!failure_reason.empty()) {
        finish_egress(goal_handle, rclcpp_action::ResultCode::ABORTED, failure_reason);
        return;
      }
      const double remaining = std::max(0.0, egress_distance_ - travel);
      if (!scan_corridor_clear(remaining, failure_reason)) {
        finish_egress(goal_handle, rclcpp_action::ResultCode::ABORTED, failure_reason);
        return;
      }
      if (evidence_ok && travel >= egress_distance_) {
        finish_egress(goal_handle, rclcpp_action::ResultCode::SUCCEEDED, "SUCCEEDED");
        return;
      }
      auto feedback = std::make_shared<BackUp::Feedback>();
      feedback->distance_traveled = static_cast<float>(std::max(0.0, travel));
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(20ms);
    }
    finish_egress(goal_handle, rclcpp_action::ResultCode::ABORTED, "ROS_SHUTDOWN");
  }

  void publish(double linear, double angular) {
    geometry_msgs::msg::TwistStamped request;
    request.header.stamp = now();
    request.header.frame_id = "base_footprint";
    request.twist.linear.x = linear;
    request.twist.angular.z = angular;
    request_pub_->publish(request);
  }

  std::chrono::milliseconds source_timeout_{200};
  std::chrono::milliseconds manipulator_status_timeout_{200};
  std::chrono::milliseconds egress_status_timeout_{200};
  std::chrono::milliseconds egress_odometry_timeout_{200};
  std::chrono::milliseconds egress_scan_timeout_{1000};
  double output_frequency_{20.0};
  double max_linear_velocity_{0.5};
  double max_angular_velocity_{0.4};
  double max_linear_acceleration_{0.5};
  double max_angular_acceleration_{0.4};
  double egress_max_distance_{0.50};
  double egress_max_speed_{0.10};
  double egress_time_limit_s_{60.0};
  double egress_drift_tolerance_{0.05};
  double egress_yaw_tolerance_{0.05};
  double egress_clearance_{0.05};
  std::string egress_footprint_text_;
  std::vector<Point2> egress_footprint_;

  bool have_source_{false};
  bool source_valid_{false};
  bool source_active_{false};
  bool require_manipulator_stowed_{false};
  bool have_manipulator_status_{false};
  bool manipulator_status_valid_{false};
  bool have_base_status_{false};
  bool have_odometry_{false};
  bool have_rear_scan_{false};
  bool egress_running_{false};
  bool lifecycle_deactivating_{false};
  uint32_t manipulator_boot_id_{0};
  uint32_t manipulator_sequence_{0};
  double last_linear_{0.0};
  double last_angular_{0.0};
  double egress_distance_{0.0};
  double egress_speed_{0.0};
  double egress_allowance_s_{0.0};
  double egress_start_yaw_{0.0};
  geometry_msgs::msg::Twist source_command_;
  geometry_msgs::msg::Pose egress_start_odom_;
  amr_interfaces::msg::ManipulatorStatus last_manipulator_message_;
  amr_interfaces::msg::BaseStatus base_status_;
  nav_msgs::msg::Odometry odometry_;
  sensor_msgs::msg::LaserScan rear_scan_;
  SteadyTime last_source_{};
  SteadyTime last_output_{};
  SteadyTime last_manipulator_status_{};
  SteadyTime last_base_status_{};
  SteadyTime last_odometry_{};
  SteadyTime last_rear_scan_{};
  SteadyTime egress_started_{};
  SteadyTime suppress_source_until_{};
  std::mutex state_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr source_sub_;
  rclcpp::Subscription<amr_interfaces::msg::ManipulatorStatus>::SharedPtr manipulator_status_sub_;
  rclcpp::Subscription<amr_interfaces::msg::BaseStatus>::SharedPtr base_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
  rclcpp_action::Server<BackUp>::SharedPtr egress_action_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace amr_control

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_control::CommandArbitrationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
