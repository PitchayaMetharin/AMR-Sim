#include <algorithm>
#include <deque>
#include <memory>
#include <string>

#include "amr_interfaces/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace amr_sensor_adapters {

class ProductCameraAdapterNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  ProductCameraAdapterNode() : LifecycleNode("product_camera_adapter_node") {
    const std::string raw_prefix = "/amr/simulation/sensors/product_camera";
    const std::string stable_prefix = "/amr/sensors/product_camera";
    raw_image_topic_ = declare_parameter("raw_image_topic", raw_prefix + "/image");
    raw_info_topic_ = declare_parameter("raw_camera_info_topic", raw_prefix + "/camera_info");
    raw_depth_topic_ = declare_parameter("raw_depth_topic", raw_prefix + "/depth_image");
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      stable_prefix + "/image_rect", amr_interfaces::qos::sensor());
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      stable_prefix + "/camera_info", amr_interfaces::qos::sensor());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      stable_prefix + "/depth", amr_interfaces::qos::sensor());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::Image::SharedPtr message) {
        if (valid_dimensions(*message)) {
          pending_images_.push_back(std::move(message));
          trim(pending_images_);
          publish_synchronized_pair();
        }
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_info_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {
        if (valid_dimensions(*message)) {
          pending_infos_.push_back(std::move(message));
          trim(pending_infos_);
          publish_synchronized_pair();
        }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_depth_topic_, amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::Image::SharedPtr message) {
        if (!depth_pub_->is_activated() || !valid_dimensions(*message)) {
          return;
        }
        message->header.frame_id = "product_camera_optical_frame";
        depth_pub_->publish(*message);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    image_pub_->on_activate();
    info_pub_->on_activate();
    depth_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    image_pub_->on_deactivate();
    info_pub_->on_deactivate();
    depth_pub_->on_deactivate();
    pending_images_.clear();
    pending_infos_.clear();
    return CallbackReturn::SUCCESS;
  }

 private:
  static bool valid_dimensions(const sensor_msgs::msg::Image & message) {
    return message.width == 640U && message.height == 480U;
  }

  static bool valid_dimensions(const sensor_msgs::msg::CameraInfo & message) {
    return message.width == 640U && message.height == 480U;
  }

  static bool equal_stamps(
    const builtin_interfaces::msg::Time & left,
    const builtin_interfaces::msg::Time & right)
  {
    return left.sec == right.sec && left.nanosec == right.nanosec;
  }

  template<typename MessageType>
  static void trim(std::deque<std::shared_ptr<MessageType>> & messages) {
    constexpr std::size_t maximum_pending_messages = 20U;
    while (messages.size() > maximum_pending_messages) {
      messages.pop_front();
    }
  }

  void publish_synchronized_pair() {
    for (auto image = pending_images_.begin(); image != pending_images_.end(); ++image) {
      const auto info = std::find_if(
        pending_infos_.begin(), pending_infos_.end(),
        [&image](const sensor_msgs::msg::CameraInfo::SharedPtr & candidate) {
          return equal_stamps((*image)->header.stamp, candidate->header.stamp);
        });
      if (info == pending_infos_.end()) {
        continue;
      }
      if (image_pub_->is_activated() && info_pub_->is_activated()) {
        (*image)->header.frame_id = "product_camera_optical_frame";
        (*info)->header.frame_id = "product_camera_optical_frame";
        image_pub_->publish(**image);
        info_pub_->publish(**info);
      }
      pending_infos_.erase(info);
      pending_images_.erase(image);
      return;
    }
  }

  std::string raw_image_topic_;
  std::string raw_info_topic_;
  std::string raw_depth_topic_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> pending_images_;
  std::deque<sensor_msgs::msg::CameraInfo::SharedPtr> pending_infos_;
};

}  // namespace amr_sensor_adapters

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_sensor_adapters::ProductCameraAdapterNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
