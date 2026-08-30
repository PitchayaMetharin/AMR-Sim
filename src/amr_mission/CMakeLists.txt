cmake_minimum_required(VERSION 3.8)
project(amr_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mission_supervisor_node src/mission_supervisor_node.cpp)
target_include_directories(mission_supervisor_node PRIVATE include)
ament_target_dependencies(
  mission_supervisor_node nav2_msgs nav_msgs rclcpp_action rclcpp_lifecycle tf2 tf2_ros)
install(TARGETS mission_supervisor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_goal_validation test/test_goal_validation.cpp)
  target_include_directories(test_goal_validation PRIVATE include)
  ament_target_dependencies(test_goal_validation geometry_msgs)
  ament_add_gtest(
    test_mission_supervisor_behavior
    test/test_mission_supervisor_behavior.cpp)
  target_include_directories(test_mission_supervisor_behavior PRIVATE include)
  ament_target_dependencies(
    test_mission_supervisor_behavior
    lifecycle_msgs nav2_msgs nav_msgs rclcpp_action rclcpp_lifecycle tf2 tf2_ros)
  set_tests_properties(
    test_mission_supervisor_behavior PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(mission_contract_test test/test_mission_contract.py)
endif()

ament_package()
