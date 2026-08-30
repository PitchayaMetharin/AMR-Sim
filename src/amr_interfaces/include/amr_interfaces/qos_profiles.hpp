#pragma once

#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"

namespace amr_interfaces::qos {

inline rclcpp::QoS sensor() {
  return rclcpp::SensorDataQoS();
}

inline rclcpp::QoS state() {
  return rclcpp::QoS(rclcpp::KeepLast(5))
      .reliable()
      .durability_volatile();
}

inline rclcpp::QoS authority() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile()
      .deadline(rclcpp::Duration::from_seconds(0.1));
}

inline rclcpp::QoS command() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile()
      .deadline(rclcpp::Duration::from_seconds(0.1))
      .lifespan(rclcpp::Duration::from_seconds(0.2));
}

inline rclcpp::QoS nav2_command_input() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();
}

inline rclcpp::QoS diagnostic() {
  return rclcpp::QoS(rclcpp::KeepLast(20))
      .reliable()
      .durability_volatile();
}

}  // namespace amr_interfaces::qos
