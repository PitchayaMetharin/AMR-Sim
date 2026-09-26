#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/WaitHelpers.hh>

#include <rclcpp/rclcpp.hpp>

namespace portable_robot_inserter
{

using namespace std::chrono_literals;

// Keep insertion within the existing bounded 30 s controller startup budget.
constexpr unsigned int kRequestTimeoutMs = 30000;
constexpr auto kServiceProbeInterval = 1s;
constexpr auto kServiceReportInterval = 60s;

struct SpawnArguments
{
  std::string world;
  std::string name;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

bool valid_world_name(const std::string &world)
{
  if (world.empty()) {
    return false;
  }
  for (const char character : world) {
    if (!((character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') || character == '_'))
    {
      return false;
    }
  }
  return true;
}

bool valid_entity_name(const std::string &name)
{
  return !name.empty() && name.find('/') == std::string::npos &&
         name.find('\\') == std::string::npos;
}

bool parse_finite_double(
  const std::string &text,
  double &value,
  std::string &error,
  const std::string &label)
{
  try {
    std::size_t consumed = 0;
    value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
      error = label + " must be a finite number";
      return false;
    }
  } catch (const std::exception &) {
    error = label + " must be a finite number";
    return false;
  }
  return true;
}

bool parse_arguments(
  const std::vector<std::string> &arguments,
  SpawnArguments &output,
  std::string &error)
{
  bool have_world = false;
  bool have_name = false;
  bool have_x = false;
  bool have_y = false;
  bool have_z = false;
  bool have_yaw = false;

  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const auto &option = arguments[index];
    if (index + 1 >= arguments.size()) {
      error = "missing value for " + option;
      return false;
    }
    const auto &value = arguments[++index];
    if (option == "--world") {
      if (have_world) {
        error = "duplicate --world";
        return false;
      }
      output.world = value;
      have_world = true;
    } else if (option == "--name") {
      if (have_name) {
        error = "duplicate --name";
        return false;
      }
      output.name = value;
      have_name = true;
    } else if (option == "--x" || option == "--y" || option == "--z" ||
      option == "--yaw")
    {
      double *target = nullptr;
      bool *seen = nullptr;
      std::string label;
      if (option == "--x") {
        target = &output.x;
        seen = &have_x;
        label = "x";
      } else if (option == "--y") {
        target = &output.y;
        seen = &have_y;
        label = "y";
      } else if (option == "--z") {
        target = &output.z;
        seen = &have_z;
        label = "z";
      } else {
        target = &output.yaw;
        seen = &have_yaw;
        label = "yaw";
      }
      if (*seen) {
        error = "duplicate " + option;
        return false;
      }
      if (!parse_finite_double(value, *target, error, label)) {
        return false;
      }
      *seen = true;
    } else {
      error = "unknown option: " + option;
      return false;
    }
  }

  if (!have_world || !valid_world_name(output.world)) {
    error = "world must be a non-empty [A-Za-z0-9_] name";
    return false;
  }
  if (!have_name || !valid_entity_name(output.name)) {
    error = "name must be a non-empty entity name without path separators";
    return false;
  }
  if (!have_x || !have_y || !have_z || !have_yaw) {
    error = "x, y, z, and yaw are required";
    return false;
  }
  return true;
}

std::string spawn_service_name(const std::string &world)
{
  if (!valid_world_name(world)) {
    throw std::invalid_argument("invalid world name");
  }
  return "/world/" + world + "/create/blocking";
}

gz::msgs::EntityFactory build_request(
  const SpawnArguments &arguments,
  const std::string &robot_sdf)
{
  if (!valid_world_name(arguments.world) ||
    !valid_entity_name(arguments.name) || robot_sdf.empty() ||
    !std::isfinite(arguments.x) || !std::isfinite(arguments.y) ||
    !std::isfinite(arguments.z) || !std::isfinite(arguments.yaw))
  {
    throw std::invalid_argument("invalid robot insertion request");
  }

  gz::msgs::EntityFactory request;
  request.set_name(arguments.name);
  request.set_sdf(robot_sdf);
  request.set_allow_renaming(false);
  request.set_relative_to("world");

  auto *pose = request.mutable_pose();
  pose->mutable_position()->set_x(arguments.x);
  pose->mutable_position()->set_y(arguments.y);
  pose->mutable_position()->set_z(arguments.z);
  const double half_yaw = arguments.yaw / 2.0;
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(std::sin(half_yaw));
  pose->mutable_orientation()->set_w(std::cos(half_yaw));
  return request;
}

bool insertion_succeeded(
  const bool request_completed,
  const bool service_result,
  const gz::msgs::Boolean &reply)
{
  return request_completed && service_result && reply.data();
}

bool wait_for_create_service(
  const gz::transport::Node &node,
  const std::string &service,
  const rclcpp::Logger &logger)
{
  auto next_report = std::chrono::steady_clock::now() + kServiceReportInterval;
  while (rclcpp::ok()) {
    if (gz::transport::waitForService(node, service, kServiceProbeInterval)) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_report) {
      RCLCPP_WARN(
        logger,
        "still waiting for Gazebo Transport service %s",
        service.c_str());
      next_report = now + kServiceReportInterval;
    }
  }
  RCLCPP_ERROR(logger, "ROS shutdown occurred before %s was discovered", service.c_str());
  return false;
}

}  // namespace portable_robot_inserter

#ifndef PORTABLE_ROBOT_INSERTER_TEST
int main(int argc, char **argv)
{
  using namespace portable_robot_inserter;

  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("portable_robot_inserter");
  const auto finish = [](const int code) {
      rclcpp::shutdown();
      return code;
    };

  const auto ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
  const std::vector<std::string> arguments(
    ros_arguments.begin() + (ros_arguments.empty() ? 0 : 1), ros_arguments.end());
  SpawnArguments spawn_arguments;
  std::string parse_error;
  if (!parse_arguments(arguments, spawn_arguments, parse_error)) {
    RCLCPP_ERROR(logger, "refusing to insert robot: %s", parse_error.c_str());
    return finish(2);
  }

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("portable_robot_inserter", options);
  if (!node->has_parameter("robot_description")) {
    node->declare_parameter<std::string>("robot_description", "");
  }
  const auto robot_sdf = node->get_parameter("robot_description").as_string();
  if (robot_sdf.empty()) {
    RCLCPP_ERROR(logger, "refusing to insert robot: robot_description is empty");
    return finish(2);
  }

  const auto service = spawn_service_name(spawn_arguments.world);
  gz::transport::Node gz_node;
  if (!wait_for_create_service(gz_node, service, logger)) {
    RCLCPP_ERROR(logger, "refusing to start controllers: %s was not discovered", service.c_str());
    return finish(1);
  }

  const auto request = build_request(spawn_arguments, robot_sdf);
  gz::msgs::Boolean reply;
  bool service_result = false;
  const bool request_completed = gz_node.Request(
    service, request, kRequestTimeoutMs, reply, service_result);
  if (!request_completed) {
    RCLCPP_ERROR(
      logger,
      "refusing to start controllers: Gazebo insertion request timed out after %u ms",
      kRequestTimeoutMs);
    return finish(1);
  }
  if (!insertion_succeeded(request_completed, service_result, reply)) {
    RCLCPP_ERROR(
      logger,
      "refusing to start controllers: Gazebo rejected robot insertion (transport=%s, response=%s)",
      service_result ? "ok" : "failed", reply.data() ? "success" : "failure");
    return finish(1);
  }

  RCLCPP_INFO(logger, "robot %s inserted into world %s", spawn_arguments.name.c_str(),
    spawn_arguments.world.c_str());
  return finish(0);
}
#endif
