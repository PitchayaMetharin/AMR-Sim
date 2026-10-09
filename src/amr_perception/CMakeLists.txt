cmake_minimum_required(VERSION 3.8)
project(amr_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

function(add_lidar_pipeline target sensor_id)
  add_executable(${target} src/lidar_pipeline_node.cpp)
  target_include_directories(${target} PRIVATE include)
  target_compile_definitions(${target} PRIVATE AMR_SENSOR_ID="${sensor_id}")
  ament_target_dependencies(${target} amr_interfaces rclcpp_lifecycle sensor_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_lidar_pipeline(front_lidar_perception_node front_lidar)
add_lidar_pipeline(rear_lidar_perception_node rear_lidar)

add_executable(lidar_scan_merger_node src/lidar_scan_merger_node.cpp)
target_include_directories(lidar_scan_merger_node PRIVATE include)
ament_target_dependencies(
  lidar_scan_merger_node amr_interfaces rclcpp_lifecycle sensor_msgs tf2 tf2_ros)
install(TARGETS lidar_scan_merger_node DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/perception_fault_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_fault_acceptance)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_point_cloud_validation test/test_point_cloud_validation.cpp)
  target_include_directories(test_point_cloud_validation PRIVATE include)
  ament_target_dependencies(test_point_cloud_validation sensor_msgs)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    test_lidar_pipeline_configuration
    test/test_lidar_pipeline_configuration.cpp)
  target_include_directories(
    test_lidar_pipeline_configuration PRIVATE include)
  ament_target_dependencies(
    test_lidar_pipeline_configuration
    amr_interfaces lifecycle_msgs rclcpp_lifecycle sensor_msgs)
  set_tests_properties(
    test_lidar_pipeline_configuration PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(perception_contract_test test/test_perception_contract.py)
  ament_add_gtest(test_scan_merge test/test_scan_merge.cpp)
  target_include_directories(test_scan_merge PRIVATE include)
  ament_target_dependencies(test_scan_merge sensor_msgs)
  ament_add_pytest_test(dual_lidar_wiring_test test/test_dual_lidar_wiring.py)
endif()

ament_package()
