cmake_minimum_required(VERSION 3.8)
project(amr_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_include_directories(wheel_odometry_node PRIVATE include)
ament_target_dependencies(
  wheel_odometry_node amr_interfaces geometry_msgs nav_msgs rclcpp_lifecycle sensor_msgs)

install(TARGETS wheel_odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_diff_drive test/test_diff_drive.cpp)
  target_include_directories(test_diff_drive PRIVATE include)
  ament_add_gtest(
    test_wheel_odometry_configuration
    test/test_wheel_odometry_configuration.cpp)
  target_include_directories(
    test_wheel_odometry_configuration PRIVATE include)
  ament_target_dependencies(
    test_wheel_odometry_configuration
    amr_interfaces geometry_msgs lifecycle_msgs nav_msgs
    rclcpp_lifecycle sensor_msgs)
  set_tests_properties(
    test_wheel_odometry_configuration PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(test_localization_contract test/test_localization_contract.py)
endif()

ament_package()
