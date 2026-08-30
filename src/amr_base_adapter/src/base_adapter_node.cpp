#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace amr_base_adapter {
// Final ROS-side adapter before Gazebo.  It has no authority to choose a
// motion source or grant permission; it only accepts a valid gated command,
// bridges it into the plant, and republishes raw plant feedback.
class BaseAdapterNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit BaseAdapterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("base_adapter_node", options) {
    // Topic names and receive-time deadlines remain parameters so simulation
    // wiring can be explicit, while defaults preserve the ownership contract.
    gated_command_topic_ = declare_parameter("gated_command_topic", "/amr/control/cmd_vel");
    simulation_command_topic_ = declare_parameter("simulation_command_topic", "/amr/simulation/base/cmd_vel");
    raw_odometry_topic_ = declare_parameter("raw_odometry_topic", "/amr/simulation/base/odometry");
    raw_joint_states_topic_ = declare_parameter("raw_joint_states_topic", "/amr/simulation/base/joint_states");
    gated_command_timeout_ = std::chrono::milliseconds(
      declare_parameter("gated_command_timeout_ms", 200));
    input_timeout_ = std::chrono::milliseconds(
      declare_parameter("input_timeout_ms", 300));
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Refuse an invalid timeout rather than running an adapter that cannot
    // reliably decide when it must stop forwarding a command.
    if (gated_command_timeout_.count() <= 0 || input_timeout_.count() <= 0) {
      RCLCPP_ERROR(get_logger(), "Adapter timeouts must be positive");
      return CallbackReturn::FAILURE;
    }
    // This publisher is bridged to Gazebo's native base command topic.
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      simulation_command_topic_, amr_interfaces::qos::command());
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/amr/base/odometry_raw", amr_interfaces::qos::sensor());
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/amr/base/joint_states", amr_interfaces::qos::sensor());
    status_pub_ = create_publisher<amr_interfaces::msg::BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic());
    // Record the newest constrained command.  It is revalidated and expires in the
    // timer callback instead of being forwarded directly from this callback.
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      gated_command_topic_, amr_interfaces::qos::command(),
      [this](geometry_msgs::msg::TwistStamped::SharedPtr message) {
        gated_command_ = *message;
        gated_command_valid_ = valid_command(*message);
        have_gated_command_ = true;
        last_gated_command_ = std::chrono::steady_clock::now();
      });
    // Raw plant feedback is passed through unchanged under stable AMR names.
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odometry_topic_, amr_interfaces::qos::sensor(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        have_odometry_ = true;
        last_odometry_ = std::chrono::steady_clock::now();
        odometry_pub_->publish(*message);
      });
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      raw_joint_states_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::JointState::SharedPtr message) {
        have_joint_states_ = true;
        last_joint_states_ = std::chrono::steady_clock::now();
        joint_states_pub_->publish(*message);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    // Start with no acceptable command or feedback; then produce a zero
    // command/status immediately and every 50 ms thereafter.
    command_pub_->on_activate(); odometry_pub_->on_activate(); joint_states_pub_->on_activate(); status_pub_->on_activate();
    boot_id_ = static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    sequence_ = 0;
    have_gated_command_ = false;
    gated_command_valid_ = false;
    have_odometry_ = false;
    have_joint_states_ = false;
    command_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { publish_command(); });
    publish_command();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    // Stop periodic work, clear cached evidence, and attempt a final zero
    // while the lifecycle publisher is still active.
    command_timer_.reset();
    have_gated_command_ = false;
    have_odometry_ = false;
    have_joint_states_ = false;
    publish_command();
    command_pub_->on_deactivate(); odometry_pub_->on_deactivate(); joint_states_pub_->on_deactivate(); status_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }
 private:
  static bool valid_command(const geometry_msgs::msg::TwistStamped & command) {
    // Enforce the stamped base-frame planar-command contract a second time at
    // the boundary to the simulator.
    const auto finite = [](double value) { return std::isfinite(value); };
    return command.header.frame_id == "base_footprint" &&
      finite(command.twist.linear.x) && finite(command.twist.linear.y) &&
      finite(command.twist.linear.z) && finite(command.twist.angular.x) &&
      finite(command.twist.angular.y) && finite(command.twist.angular.z) &&
      command.twist.linear.y == 0.0 && command.twist.linear.z == 0.0 &&
      command.twist.angular.x == 0.0 && command.twist.angular.y == 0.0;
  }
  void publish_command() {
    geometry_msgs::msg::TwistStamped command;
    // Only a recent valid control output is allowed through; default-constructed
    // TwistStamped is therefore the intentional stopped command.
    const bool fresh = have_gated_command_ && gated_command_valid_ &&
      std::chrono::steady_clock::now() - last_gated_command_ <=
      gated_command_timeout_;
    if (fresh) command = gated_command_;
    command.header.stamp = now();
    command.header.frame_id = "base_footprint";
    command_pub_->publish(command);

    const auto steady_now = std::chrono::steady_clock::now();
    // Health evidence is separate from command forwarding.  It says whether
    // both feedback streams have arrived within their receive-time deadline.
    const bool odometry_fresh =
      have_odometry_ && steady_now - last_odometry_ <= input_timeout_;
    const bool joint_states_fresh =
      have_joint_states_ && steady_now - last_joint_states_ <= input_timeout_;
    amr_interfaces::msg::BaseStatus status;
    status.header.stamp = now();
    status.sequence = ++sequence_;
    status.valid = odometry_fresh && joint_states_fresh;
    status.source_boot_id = boot_id_;
    status.state = status.valid ? status.READY : status.UNAVAILABLE;
    status.reason = status.REASON_READY;
    if (!odometry_fresh) {
      status.reason = status.REASON_ODOMETRY_MISSING_OR_STALE;
    } else if (!joint_states_fresh) {
      status.reason = status.REASON_JOINT_STATES_MISSING_OR_STALE;
    }
    status_pub_->publish(status);
  }
  std::string gated_command_topic_, simulation_command_topic_, raw_odometry_topic_, raw_joint_states_topic_;
  std::chrono::milliseconds gated_command_timeout_{200};
  std::chrono::milliseconds input_timeout_{300};
  std::chrono::steady_clock::time_point last_gated_command_{};
  std::chrono::steady_clock::time_point last_odometry_{};
  std::chrono::steady_clock::time_point last_joint_states_{};
  bool have_gated_command_{false}, gated_command_valid_{false};
  bool have_odometry_{false}, have_joint_states_{false};
  uint32_t boot_id_{0}, sequence_{0};
  geometry_msgs::msg::TwistStamped gated_command_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_interfaces::msg::BaseStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};
}  // namespace amr_base_adapter
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_base_adapter::BaseAdapterNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
