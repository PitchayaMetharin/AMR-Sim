cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

add_executable(phase14_collision_check test/phase14_collision_check.cpp)
find_package(moveit_core REQUIRED)
target_link_libraries(phase14_collision_check
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
)
install(TARGETS phase14_collision_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_test test/test_description.py)
endif()

ament_package()
