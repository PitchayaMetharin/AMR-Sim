#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>

namespace amr_factory {

class GazeboControlWorldProxy final : public rclcpp::Node {
 public:
  GazeboControlWorldProxy()
      : Node("gazebo_control_world_proxy") {
    gazebo_service_ = declare_parameter<std::string>(
        "gazebo_service", "/world/factory_world/control");
    request_timeout_ms_ = declare_parameter<int>("request_timeout_ms", 3000);
    if (gazebo_service_.empty() || request_timeout_ms_ <= 0) {
      throw std::invalid_argument(
          "gazebo_service must be non-empty and request_timeout_ms must be positive");
    }

    service_ = create_service<ros_gz_interfaces::srv::ControlWorld>(
        "/world/factory_world/control",
        std::bind(
            &GazeboControlWorldProxy::handle_request, this,
            std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
        get_logger(), "Forwarding ROS ControlWorld to Gazebo service [%s]",
        gazebo_service_.c_str());
  }

 private:
  void handle_request(
      const std::shared_ptr<ros_gz_interfaces::srv::ControlWorld::Request> request,
      std::shared_ptr<ros_gz_interfaces::srv::ControlWorld::Response> response) {
    response->success = false;
    const auto &world_control = request->world_control;

    // This proxy is deliberately limited to pause and one-step control for
    // Gate 6 startup.  In particular, do not pass ROS's default zero
    // run_to_sim_time through ros_gz_bridge: that bridge serializes the field
    // as present, which makes Gazebo remain paused at the requested time.
    if (world_control.reset.all || world_control.reset.time_only ||
        world_control.reset.model_only || world_control.seed != 0 ||
        world_control.run_to_sim_time.sec != 0 ||
        world_control.run_to_sim_time.nanosec != 0 ||
        world_control.multi_step > 1 ||
        (world_control.multi_step != 0 && !world_control.step)) {
      RCLCPP_ERROR(get_logger(), "Rejected unsupported world-control request");
      return;
    }

    gz::msgs::WorldControl gazebo_request;
    gazebo_request.set_pause(world_control.pause);
    if (world_control.step) {
      gazebo_request.set_step(true);
      gazebo_request.set_multi_step(world_control.multi_step == 0 ? 1 : world_control.multi_step);
    }

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
      RCLCPP_ERROR(get_logger(), "Gazebo world-control request could not be dispatched");
      return;
    }
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      if (!state->condition.wait_for(
              lock, std::chrono::milliseconds(request_timeout_ms_),
              [state]() { return state->completed; })) {
        RCLCPP_ERROR(get_logger(), "Gazebo world-control request timed out");
        return;
      }
      response->success = state->result && state->success;
    }
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "Gazebo rejected the world-control request");
    }
  }

  std::string gazebo_service_;
  int request_timeout_ms_{0};
  gz::transport::Node gazebo_node_;
  rclcpp::Service<ros_gz_interfaces::srv::ControlWorld>::SharedPtr service_;
};

}  // namespace amr_factory

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_factory::GazeboControlWorldProxy>());
  } catch (const std::exception &error) {
    fprintf(stderr, "gazebo_control_world_proxy: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
