cmake_minimum_required(VERSION 3.8)
project(amr_health)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(health_supervisor_node src/health_supervisor_node.cpp)
ament_target_dependencies(
  health_supervisor_node amr_interfaces rclcpp_lifecycle)

install(TARGETS health_supervisor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    health_configuration_test test/test_health_configuration.cpp)
  ament_target_dependencies(
    health_configuration_test
    amr_interfaces lifecycle_msgs rclcpp_lifecycle)
  set_tests_properties(
    health_configuration_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(health_contract_test test/test_health_contract.py)
endif()

ament_package()
