#include <memory>
#include <string>

#include "amr_interfaces/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace amr_sensor_adapters {
// Generic lifecycle adapter compiled twice with a different sensor name. It
// copies simulation data without estimation, filtering, or motion authority.
class LidarAdapterNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  LidarAdapterNode() : LifecycleNode(std::string(AMR_SENSOR_NAME) + "_adapter_node"), sensor_name_(AMR_SENSOR_NAME) {
    // Keep simulator-facing names separate from stable project-facing names.
    const auto raw_prefix = "/amr/simulation/sensors/" + sensor_name_;
    const auto output_prefix = "/amr/sensors/" + sensor_name_;
    raw_scan_topic_ = declare_parameter("raw_scan_topic", raw_prefix + "/scan");
    raw_points_topic_ = declare_parameter("raw_points_topic", raw_prefix + "/points");
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_prefix + "/scan", amr_interfaces::qos::sensor());
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_prefix + "/points", amr_interfaces::qos::sensor());
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Each subscription relays its matching message type unchanged.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      raw_scan_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) { scan_pub_->publish(*message); });
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_points_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr message) { points_pub_->publish(*message); });
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override { scan_pub_->on_activate(); points_pub_->on_activate(); return CallbackReturn::SUCCESS; }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override { scan_pub_->on_deactivate(); points_pub_->on_deactivate(); return CallbackReturn::SUCCESS; }
 private:
  std::string sensor_name_, raw_scan_topic_, raw_points_topic_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
};
}  // namespace amr_sensor_adapters
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sensor_adapters::LidarAdapterNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
