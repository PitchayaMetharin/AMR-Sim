cmake_minimum_required(VERSION 3.8)
project(amr_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ManipulateProduct.action"
  "action/TransportProduct.action"
  "msg/BaseStatus.msg"
  "msg/FactoryStatus.msg"
  "msg/HealthStatus.msg"
  "msg/ManipulatorStatus.msg"
  "srv/SetOperationMode.srv"
  DEPENDENCIES std_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(fail_closed_defaults_test
    test/test_fail_closed_defaults.cpp
  )
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  target_link_libraries(fail_closed_defaults_test "${cpp_typesupport_target}")

  ament_add_gtest(qos_profiles_test test/test_qos_profiles.cpp)
  target_include_directories(qos_profiles_test PRIVATE include)
  ament_target_dependencies(qos_profiles_test rclcpp)

  ament_add_pytest_test(interface_schema_test test/test_interface_schema.py)
endif()

ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
