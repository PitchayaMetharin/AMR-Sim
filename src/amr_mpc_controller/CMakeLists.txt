cmake_minimum_required(VERSION 3.8)
project(amr_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amr_interfaces REQUIRED)
add_library(heading_latched_rpp SHARED src/heading_latched_rpp.cpp src/final_position_rpp.cpp)
target_compile_features(heading_latched_rpp PUBLIC cxx_std_17)
target_include_directories(heading_latched_rpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(heading_latched_rpp
  nav2_regulated_pure_pursuit_controller nav2_core pluginlib tf2 nav2_costmap_2d
  nav2_util rclcpp rclcpp_lifecycle tf2_ros geometry_msgs nav_msgs amr_interfaces)
pluginlib_export_plugin_description_file(nav2_core heading_latched_rpp_plugin.xml)
install(TARGETS heading_latched_rpp LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_controller REQUIRED)
  ament_add_gtest(heading_latched_rpp_test test/test_heading_latched_rpp.cpp)
  target_link_libraries(heading_latched_rpp_test heading_latched_rpp)
  ament_target_dependencies(heading_latched_rpp_test nav2_controller)
  ament_add_gtest(final_position_rpp_test test/test_final_position_rpp.cpp)
  target_link_libraries(final_position_rpp_test heading_latched_rpp)
  ament_target_dependencies(final_position_rpp_test nav2_controller amr_interfaces)
  set_tests_properties(final_position_rpp_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=232;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  set_tests_properties(heading_latched_rpp_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=232;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(mpc_controller_contract_test
    test/test_mpc_controller_contract.py)
endif()

ament_package()
