#include <memory>
#include <string>

#include "amr_interfaces/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace amr_sensor_adapters {
// Copies the simulated IMU to the stable AMR boundary. Filtering and pose
// estimation are deliberately downstream responsibilities.
class ImuAdapterNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  ImuAdapterNode() : LifecycleNode("imu_adapter_node") {
    // Parameters make simulator input and stable public output explicit.
    input_topic_ = declare_parameter("raw_imu_topic", "/amr/simulation/sensors/imu/data");
    output_topic_ = declare_parameter("imu_topic", "/amr/sensors/imu/data_raw");
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Relay high-rate raw IMU data with the sensor QoS profile.
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, amr_interfaces::qos::sensor());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::Imu::SharedPtr message) { publisher_->publish(*message); });
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override { publisher_->on_activate(); return CallbackReturn::SUCCESS; }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override { publisher_->on_deactivate(); return CallbackReturn::SUCCESS; }
 private:
  std::string input_topic_, output_topic_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};
}  // namespace amr_sensor_adapters
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sensor_adapters::ImuAdapterNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
