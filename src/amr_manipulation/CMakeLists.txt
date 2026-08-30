cmake_minimum_required(VERSION 3.8)
project(amr_manipulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(manipulation_validation
  src/attachment_gate.cpp
  src/tag_observation_validator.cpp)
target_include_directories(manipulation_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gate6_empty_motion src/gate6_empty_motion.cpp)
ament_target_dependencies(gate6_empty_motion
  amr_interfaces
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  nav2_msgs
  nav_msgs
  rclcpp
  shape_msgs)

add_executable(gate6_mass_stage src/gate6_mass_stage.cpp)
ament_target_dependencies(gate6_mass_stage
  amr_interfaces
  control_msgs
  moveit_msgs
  moveit_ros_planning_interface
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  moveit_core
  ros_gz_interfaces
  shape_msgs
  std_msgs
  trajectory_msgs)
target_link_libraries(gate6_mass_stage manipulation_validation)

add_executable(manipulation_supervisor_node src/manipulation_supervisor_node.cpp)
ament_target_dependencies(manipulation_supervisor_node
  amr_interfaces
  rclcpp
  rclcpp_action)

install(TARGETS manipulation_validation EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gate6_empty_motion gate6_mass_stage manipulation_supervisor_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/gate6_product_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_product_test)
install(PROGRAMS
  scripts/gate6_evidence_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_evidence_analyzer)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(tag_observation_validator_test
    test/tag_observation_validator_test.cpp)
  target_link_libraries(tag_observation_validator_test manipulation_validation)
  ament_add_gtest(attachment_gate_test test/attachment_gate_test.cpp)
  target_link_libraries(attachment_gate_test manipulation_validation)
  ament_add_pytest_test(moveit_config_test test/test_moveit_config.py)
  ament_add_pytest_test(manipulation_contract_test test/test_manipulation_contract.py)
  ament_add_pytest_test(product_test_contract_test test/test_product_test_contract.py)
  ament_add_pytest_test(gate6_completion_contract_test
    test/test_gate6_completion_contract.py)
endif()

ament_package()
