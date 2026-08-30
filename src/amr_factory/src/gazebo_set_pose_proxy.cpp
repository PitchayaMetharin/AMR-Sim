#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

namespace amr_factory {

class GazeboSetPoseProxy final : public rclcpp::Node {
 public:
  GazeboSetPoseProxy()
      : Node("gazebo_set_pose_proxy") {
    gazebo_service_ = declare_parameter<std::string>(
        "gazebo_service", "/world/factory_world/set_pose");
    request_timeout_ms_ = declare_parameter<int>("request_timeout_ms", 3000);
    if (gazebo_service_.empty() || request_timeout_ms_ <= 0) {
      throw std::invalid_argument(
          "gazebo_service must be non-empty and request_timeout_ms must be positive");
    }

    service_ = create_service<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/factory_world/set_pose",
        std::bind(
            &GazeboSetPoseProxy::handle_request, this,
            std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
        get_logger(), "Forwarding ROS SetEntityPose to Gazebo service [%s]",
        gazebo_service_.c_str());
  }

 private:
  static bool finite(double value) {
    return std::isfinite(value);
  }

  static bool valid_pose(const geometry_msgs::msg::Pose &pose) {
    const auto &position = pose.position;
    const auto &orientation = pose.orientation;
    if (!finite(position.x) || !finite(position.y) || !finite(position.z) ||
        !finite(orientation.x) || !finite(orientation.y) ||
        !finite(orientation.z) || !finite(orientation.w)) {
      return false;
    }
    const double norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    return finite(norm) && norm > 1e-12;
  }

  void handle_request(
      const std::shared_ptr<ros_gz_interfaces::srv::SetEntityPose::Request> request,
      std::shared_ptr<ros_gz_interfaces::srv::SetEntityPose::Response> response) {
    response->success = false;

    // This adapter is intentionally scoped to model pose reset.  It must not
    // become an unbounded way to move arbitrary Gazebo entities.
    if (request->entity.type != ros_gz_interfaces::msg::Entity::MODEL) {
      RCLCPP_ERROR(get_logger(), "Rejected set-pose request for non-model entity");
      return;
    }
    if (request->entity.name.empty() && request->entity.id == 0) {
      RCLCPP_ERROR(get_logger(), "Rejected set-pose request without model name or id");
      return;
    }
    if (!valid_pose(request->pose)) {
      RCLCPP_ERROR(get_logger(), "Rejected set-pose request with invalid pose");
      return;
    }
    if (request->entity.id > std::numeric_limits<std::uint32_t>::max()) {
      RCLCPP_ERROR(get_logger(), "Rejected set-pose request with an out-of-range entity id");
      return;
    }

    gz::msgs::Pose gazebo_request;
    if (!request->entity.name.empty()) {
      gazebo_request.set_name(request->entity.name);
    }
    if (request->entity.id != 0) {
      gazebo_request.set_id(static_cast<std::uint32_t>(request->entity.id));
    }
    auto *position = gazebo_request.mutable_position();
    position->set_x(request->pose.position.x);
    position->set_y(request->pose.position.y);
    position->set_z(request->pose.position.z);
    auto *orientation = gazebo_request.mutable_orientation();
    orientation->set_x(request->pose.orientation.x);
    orientation->set_y(request->pose.orientation.y);
    orientation->set_z(request->pose.orientation.z);
    orientation->set_w(request->pose.orientation.w);

    RCLCPP_INFO(get_logger(), "Dispatching Gazebo set-pose request for model [%s]",
                request->entity.name.c_str());
    struct RequestState {
      std::mutex mutex;
      std::condition_variable condition;
      bool completed{false};
      bool result{false};
      bool success{false};
    };
    const auto state = std::make_shared<RequestState>();
    std::function<void(const gz::msgs::Boolean &, const bool)> callback =
        [state](const gz::msgs::Boolean &gazebo_response, const bool result) {
          {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->completed = true;
            state->result = result;
            state->success = result && gazebo_response.data();
          }
          state->condition.notify_one();
        };
    if (!gazebo_node_.Request(gazebo_service_, gazebo_request, callback)) {
      RCLCPP_ERROR(get_logger(), "Gazebo set-pose request could not be dispatched");
      return;
    }
    RCLCPP_INFO(get_logger(), "Gazebo set-pose request dispatched");
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      if (!state->condition.wait_for(
              lock, std::chrono::milliseconds(request_timeout_ms_),
              [state]() { return state->completed; })) {
        RCLCPP_ERROR(get_logger(), "Gazebo set-pose request timed out");
        return;
      }
      response->success = state->result && state->success;
    }
    if (!response->success) {
      RCLCPP_ERROR(
          get_logger(), "Gazebo set-pose request failed for model [%s]",
          request->entity.name.c_str());
    }
  }

  std::string gazebo_service_;
  int request_timeout_ms_{0};
  gz::transport::Node gazebo_node_;
  rclcpp::Service<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr service_;
};

}  // namespace amr_factory

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_factory::GazeboSetPoseProxy>());
  } catch (const std::exception &error) {
    fprintf(stderr, "gazebo_set_pose_proxy: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
