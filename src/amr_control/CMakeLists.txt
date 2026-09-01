cmake_minimum_required(VERSION 3.8)
project(amr_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(command_arbitration_node src/command_arbitration_node.cpp)
ament_target_dependencies(
  command_arbitration_node
  amr_interfaces geometry_msgs lifecycle_msgs nav2_msgs nav_msgs rclcpp
  rclcpp_action rclcpp_lifecycle sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

install(
  TARGETS command_arbitration_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/prototype_teleop.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    control_configuration_test test/test_control_configuration.cpp)
  ament_target_dependencies(
    control_configuration_test
    amr_interfaces geometry_msgs lifecycle_msgs nav2_msgs nav_msgs rclcpp
    rclcpp_action rclcpp_lifecycle sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  set_tests_properties(
    control_configuration_test PROPERTIES
    ENVIRONMENT
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=211")
  ament_add_pytest_test(control_contract_test test/test_control_contract.py)
endif()

ament_package()
