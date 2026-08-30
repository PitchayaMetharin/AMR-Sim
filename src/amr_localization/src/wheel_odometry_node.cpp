#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "amr_localization/diff_drive.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace amr_localization {

// Convert encoder angles from the two drive wheels into local odometry. The
// EKF, not this node, owns the odom -> base_footprint TF transform.
class WheelOdometryNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit WheelOdometryNode(
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : LifecycleNode("wheel_odometry_node", options) {
    // These defaults describe the simulation geometry and are checked below.
    input_topic_ = declare_parameter(
        "joint_states_topic", "/amr/base/joint_states");
    output_topic_ = declare_parameter(
        "wheel_odometry_topic", "/amr/localization/wheel_odometry");
    left_joint_ = declare_parameter("left_joint", "left_wheel_joint");
    right_joint_ = declare_parameter("right_joint", "right_wheel_joint");
    wheel_radius_ = declare_parameter("wheel_radius", 0.1128);
    wheel_separation_ = declare_parameter("wheel_separation", 0.566);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Invalid wheel geometry would make every kinematic result meaningless.
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
        !std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0) {
      RCLCPP_ERROR(
          get_logger(), "Wheel geometry must be finite and positive");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        output_topic_, amr_interfaces::qos::state());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, amr_interfaces::qos::sensor(),
        [this](sensor_msgs::msg::JointState::SharedPtr message) {
          handle_joint_state(*message);
        });
    reset_state();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    publisher_->on_deactivate();
    reset_state();
    return CallbackReturn::SUCCESS;
  }

 private:
  static std::optional<double> joint_position(
      const sensor_msgs::msg::JointState & message, const std::string & name) {
    // JointState keeps names and positions in parallel arrays; find the named
    // drive joint before reading its corresponding finite position.
    const auto it = std::find(message.name.begin(), message.name.end(), name);
    if (it == message.name.end()) {
      return std::nullopt;
    }
    const auto index = static_cast<std::size_t>(
        std::distance(message.name.begin(), it));
    if (index >= message.position.size() ||
        !std::isfinite(message.position[index])) {
      return std::nullopt;
    }
    return message.position[index];
  }

  void reset_state() {
    // Forget previous encoders and return the integrated local pose to origin.
    initialized_ = false;
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
  }

  void handle_joint_state(const sensor_msgs::msg::JointState & message) {
    // Both encoder positions and a real timestamp are required for an update.
    const auto left = joint_position(message, left_joint_);
    const auto right = joint_position(message, right_joint_);
    const rclcpp::Time stamp(message.header.stamp);
    if (!left || !right || stamp.nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joint state lacks finite drive-wheel positions or a valid stamp");
      return;
    }

    if (!initialized_) {
      // The initial sample is only a baseline; it cannot describe motion.
      previous_left_ = *left;
      previous_right_ = *right;
      previous_stamp_ = stamp;
      initialized_ = true;
      return;
    }
    if (stamp < previous_stamp_) {
      // Time reset/reversal invalidates accumulated odometry, so restart.
      reset_state();
      previous_left_ = *left;
      previous_right_ = *right;
      previous_stamp_ = stamp;
      initialized_ = true;
      return;
    }
    if (stamp == previous_stamp_) {
      return;
    }

    // Integrate the difference between the newest and previous wheel angles.
    const double dt = (stamp - previous_stamp_).seconds();
    const double left_delta = *left - previous_left_;
    const double right_delta = *right - previous_right_;
    const auto increment = integrate_diff_drive(
        yaw_, left_delta, right_delta, wheel_radius_, wheel_separation_);
    x_ += increment.x;
    y_ += increment.y;
    yaw_ = std::atan2(
        std::sin(yaw_ + increment.yaw), std::cos(yaw_ + increment.yaw));

    // Publish the conventional local pose/velocity message in odom and base
    // frames. Orientation is the yaw-only quaternion for planar motion.
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = message.header.stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_footprint";
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    odometry.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    odometry.twist.twist.linear.x =
        wheel_radius_ * (left_delta + right_delta) / (2.0 * dt);
    odometry.twist.twist.angular.z =
        wheel_radius_ * (right_delta - left_delta) /
        (wheel_separation_ * dt);

    // Nominal simulation uncertainties only; physical values require measurement.
    odometry.pose.covariance[0] = 0.0025;
    odometry.pose.covariance[7] = 0.0025;
    odometry.pose.covariance[35] = 0.0012;
    odometry.twist.covariance[0] = 0.0025;
    odometry.twist.covariance[7] = 0.0025;
    odometry.twist.covariance[35] = 0.0012;
    publisher_->publish(odometry);

    previous_left_ = *left;
    previous_right_ = *right;
    previous_stamp_ = stamp;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string left_joint_;
  std::string right_joint_;
  double wheel_radius_{0.0};
  double wheel_separation_{0.0};
  bool initialized_{false};
  double previous_left_{0.0};
  double previous_right_{0.0};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
      publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace amr_localization

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_localization::WheelOdometryNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
