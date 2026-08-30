cmake_minimum_required(VERSION 3.8)
project(amr_base_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(base_adapter_node src/base_adapter_node.cpp)
ament_target_dependencies(base_adapter_node amr_interfaces geometry_msgs nav_msgs rclcpp_lifecycle sensor_msgs)
install(TARGETS base_adapter_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    base_adapter_node_test test/test_base_adapter_node.cpp)
  ament_target_dependencies(
    base_adapter_node_test
    amr_interfaces geometry_msgs lifecycle_msgs nav_msgs
    rclcpp_lifecycle sensor_msgs)
  set_tests_properties(
    base_adapter_node_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(
    base_adapter_contract_test test/test_base_adapter_contract.py)
endif()

ament_package()
