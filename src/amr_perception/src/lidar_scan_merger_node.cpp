#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "amr_interfaces/qos_profiles.hpp"
#include "amr_perception/scan_merge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_perception {

// Merges the front and rear LaserScans into one 360 degree scan in the base
// frame. The front scan triggers each merge; perception only, no TF
// ownership and no motion authority.
class LidarScanMergerNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit LidarScanMergerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("lidar_scan_merger_node", options) {
    output_frame_ = declare_parameter("output_frame", std::string("base_footprint"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    bin_count_ = declare_parameter("bin_count", 1440);
    range_min_ = declare_parameter("range_min", 0.2);
    range_max_ = declare_parameter("range_max", 20.5);
    max_age_sec_ = declare_parameter("max_age_sec", 0.5);
    max_skew_sec_ = declare_parameter("max_skew_sec", 0.2);
    tf_timeout_sec_ = declare_parameter("tf_timeout_sec", 0.05);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    if (output_frame_.empty() || odom_frame_.empty() || bin_count_ < 3 ||
        !std::isfinite(range_min_) || !std::isfinite(range_max_) ||
        range_min_ < 0.0 || range_max_ <= range_min_ ||
        !std::isfinite(max_age_sec_) || max_age_sec_ <= 0.0 ||
        !std::isfinite(max_skew_sec_) || max_skew_sec_ < 0.0 ||
        !std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid lidar_scan_merger_node parameters");
      return CallbackReturn::FAILURE;
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/amr/sensors/merged_lidar/scan", amr_interfaces::qos::sensor());
    rear_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/amr/sensors/rear_lidar/scan", amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {
        rear_scan_ = *message;
      });
    front_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/amr/sensors/front_lidar/scan", amr_interfaces::qos::sensor(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {
        handle_front(*message);
      });
    clear_state();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    publisher_->on_deactivate();
    clear_state();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    front_subscription_.reset();
    rear_subscription_.reset();
    publisher_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    clear_state();
    return CallbackReturn::SUCCESS;
  }

 private:
  void clear_state() {
    rear_scan_.reset();
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void drop(const char * reason) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Not publishing merged scan: %s", reason);
  }

  std::optional<SensorPose2D> sensor_pose(
    const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & output_stamp) {
    try {
      // Motion-compensated: sensor at its own stamp, expressed in the output
      // frame at the output stamp, linked through the odom frame.
      const auto transform = tf_buffer_->lookupTransform(
        output_frame_, tf2_ros::fromMsg(output_stamp),
        scan.header.frame_id, tf2_ros::fromMsg(scan.header.stamp), odom_frame_,
        tf2::durationFromSec(tf_timeout_sec_));
      const auto & q = transform.transform.rotation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return SensorPose2D{
        transform.transform.translation.x, transform.transform.translation.y, yaw};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Lidar TF lookup failed: %s", error.what());
      return std::nullopt;
    }
  }

  void handle_front(const sensor_msgs::msg::LaserScan & front) {
    if (!rear_scan_) {
      drop("no rear scan yet");
      return;
    }
    const rclcpp::Time front_stamp(front.header.stamp, RCL_ROS_TIME);
    const rclcpp::Time rear_stamp(rear_scan_->header.stamp, RCL_ROS_TIME);
    const double now_sec = get_clock()->now().seconds();
    if (!stamps_usable(
        {front_stamp.seconds(), rear_stamp.seconds()}, now_sec, max_age_sec_,
        max_skew_sec_)) {
      drop("front/rear scans stale, future, or skewed");
      return;
    }
    const rclcpp::Time output_stamp = front_stamp > rear_stamp ? front_stamp : rear_stamp;
    if (output_stamp <= last_stamp_) {
      drop("non-monotonic scan stamp");
      return;
    }
    const auto front_pose = sensor_pose(front, output_stamp);
    const auto rear_pose = sensor_pose(*rear_scan_, output_stamp);
    if (!front_pose || !rear_pose) return;

    MergedScanLayout layout;
    layout.angle_min = -M_PI;
    layout.angle_increment = 2.0 * M_PI / static_cast<double>(bin_count_);
    layout.bin_count = static_cast<std::size_t>(bin_count_);
    layout.range_min = range_min_;
    layout.range_max = range_max_;

    sensor_msgs::msg::LaserScan merged;
    if (!merge_scans(
        {ScanSource{front, *front_pose}, ScanSource{*rear_scan_, *rear_pose}},
        layout, merged)) {
      drop("merge rejected invalid scan input");
      return;
    }
    merged.header.frame_id = output_frame_;
    merged.header.stamp = output_stamp;
    publisher_->publish(merged);
    last_stamp_ = output_stamp;
  }

  std::string output_frame_;
  std::string odom_frame_;
  int bin_count_{0};
  double range_min_{0.0};
  double range_max_{0.0};
  double max_age_sec_{0.0};
  double max_skew_sec_{0.0};
  double tf_timeout_sec_{0.0};
  std::optional<sensor_msgs::msg::LaserScan> rear_scan_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_subscription_;
};

}  // namespace amr_perception

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_perception::LidarScanMergerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
