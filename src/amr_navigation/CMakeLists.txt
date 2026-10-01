cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

add_library(precision_navfn_planner SHARED src/precision_navfn_planner.cpp)
target_compile_features(precision_navfn_planner PUBLIC cxx_std_17)
target_include_directories(precision_navfn_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(precision_navfn_planner
  nav2_core nav2_costmap_2d nav2_navfn_planner nav2_util pluginlib)
pluginlib_export_plugin_description_file(nav2_core precision_planner_plugin.xml)
install(TARGETS precision_navfn_planner DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_contract_test test/test_navigation_contract.py)
  add_subdirectory(test/replay)
endif()

ament_package()
