#include <memory>
#include <cmath>
#include <string>

#include "amr_interfaces/qos_profiles.hpp"
#include "amr_perception/point_cloud_validation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace amr_perception {

// Build-time sensor ID creates one front and one rear copy of this input
// quality gate. It preserves accepted clouds and has no motion authority.
class LidarPipelineNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit LidarPipelineNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode(
      std::string(AMR_SENSOR_ID) + "_perception_node", options),
    sensor_id_(AMR_SENSOR_ID) {
    // Derive matching stable adapter/perception names from the selected sensor.
    const auto input_prefix = "/amr/sensors/" + sensor_id_;
    const auto output_prefix = "/amr/perception/" + sensor_id_;
    input_topic_ = declare_parameter("input_topic", input_prefix + "/points");
    output_topic_ = declare_parameter("output_topic", output_prefix + "/points");
    max_age_seconds_ = declare_parameter("max_age_seconds", 0.5);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // A positive finite age bound is required to detect stale cloud data.
    if (!std::isfinite(max_age_seconds_) || max_age_seconds_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(), "max_age_seconds must be finite and positive");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, amr_interfaces::qos::sensor());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, amr_interfaces::qos::sensor(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
          handle_cloud(*message);
        });
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    publisher_->on_deactivate();
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return CallbackReturn::SUCCESS;
  }

 private:
  void handle_cloud(const sensor_msgs::msg::PointCloud2 & cloud) {
    const rclcpp::Time stamp(cloud.header.stamp, RCL_ROS_TIME);
    const rclcpp::Time now = get_clock()->now();
    // Do not republish malformed, future, expired, or replayed cloud input.
    if (!has_valid_layout(cloud) || stamp > now ||
        (now - stamp).seconds() > max_age_seconds_ || stamp <= last_stamp_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping invalid, stale, future, or non-monotonic LiDAR cloud");
      return;
    }
    // Forward an accepted cloud unchanged, including its source frame.
    publisher_->publish(cloud);
    last_stamp_ = stamp;
  }

  std::string sensor_id_;
  std::string input_topic_;
  std::string output_topic_;
  double max_age_seconds_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace amr_perception

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_perception::LidarPipelineNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
