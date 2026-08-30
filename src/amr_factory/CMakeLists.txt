cmake_minimum_required(VERSION 3.8)
project(amr_factory)

find_package(ament_cmake REQUIRED)
find_package(apriltag REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(generate_apriltag_textures tools/generate_apriltag_textures.cpp)
target_link_libraries(generate_apriltag_textures apriltag::apriltag)

add_executable(factory_supervisor_node src/factory_supervisor_node.cpp)
ament_target_dependencies(factory_supervisor_node
  amr_interfaces
  rclcpp
  rclcpp_action)

add_executable(gazebo_set_pose_proxy src/gazebo_set_pose_proxy.cpp)
ament_target_dependencies(gazebo_set_pose_proxy
  geometry_msgs
  rclcpp
  ros_gz_interfaces)
target_link_libraries(gazebo_set_pose_proxy
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13)

add_executable(gazebo_control_world_proxy src/gazebo_control_world_proxy.cpp)
ament_target_dependencies(gazebo_control_world_proxy
  rclcpp
  ros_gz_interfaces)
target_link_libraries(gazebo_control_world_proxy
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13)

install(DIRECTORY assets config launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gate4_acceptance.py
  scripts/gate5_camera_acceptance.py
  scripts/factory_cli.py
  scripts/factory_runtime_preflight.py
  scripts/factory_mapping_cli.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/gate6_bootstrap_ready_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_bootstrap_ready_gate)
install(PROGRAMS scripts/gate6_bootstrap_pause_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_bootstrap_pause_gate)
install(PROGRAMS scripts/gate6_bootstrap_inserted_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_bootstrap_inserted_gate)
install(PROGRAMS scripts/gate6_controller_ready_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_controller_ready_gate)
install(PROGRAMS scripts/gate6_attachment_bootstrap.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gate6_attachment_bootstrap)
install(TARGETS generate_apriltag_textures factory_supervisor_node
  gazebo_set_pose_proxy gazebo_control_world_proxy
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(factory_assets_test test/test_factory_assets.py)
  ament_add_pytest_test(factory_supervisor_contract_test
    test/test_factory_supervisor_contract.py)
  ament_add_pytest_test(factory_demo_contract_test
    test/test_factory_demo_contract.py)
  ament_add_pytest_test(factory_runtime_preflight_test
    test/test_factory_runtime_preflight.py)
  ament_add_pytest_test(factory_mapping_cli_test
    test/test_factory_mapping_cli.py)
  ament_add_pytest_test(factory_attachment_bootstrap_contract_test
    test/test_attachment_bootstrap_contract.py)
endif()

ament_package()
