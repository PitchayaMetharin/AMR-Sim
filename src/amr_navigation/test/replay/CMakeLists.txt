cmake_minimum_required(VERSION 3.8)
project(amr_navigation_planner_replay LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)

find_library(
  NAV2_SMAC_2D_PLUGIN nav2_smac_planner_2d REQUIRED
  PATHS "${nav2_smac_planner_DIR}/../../../lib" NO_DEFAULT_PATH)
find_library(
  NAV2_SMAC_LATTICE_PLUGIN nav2_smac_planner_lattice REQUIRED
  PATHS "${nav2_smac_planner_DIR}/../../../lib" NO_DEFAULT_PATH)
find_library(
  NAV2_SMOOTHER_PLUGIN simple_smoother REQUIRED
  PATHS "${nav2_smoother_DIR}/../../../lib" NO_DEFAULT_PATH)

function(add_replay_backend target definition planner_library)
  if(ARGC GREATER 3)
    add_executable(${target} ${ARGV3})
  else()
    add_executable(${target} planner_replay_backend.cpp)
  endif()
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_definitions(${target} PRIVATE ${definition})
  ament_target_dependencies(
    ${target}
    geometry_msgs
    nav2_costmap_2d
    nav2_msgs
    nav2_util
    nav_msgs
    rclcpp
    rclcpp_lifecycle)
  target_include_directories(
    ${target}
    PRIVATE
    ${nav2_smac_planner_INCLUDE_DIRS}
    ${nav2_smoother_INCLUDE_DIRS})
  target_link_libraries(
    ${target}
    nlohmann_json::nlohmann_json
    ${planner_library}
    ${NAV2_SMOOTHER_PLUGIN})
endfunction()

add_replay_backend(planner_replay_2d AMR_REPLAY_2D_ONLY ${NAV2_SMAC_2D_PLUGIN})
add_replay_backend(planner_replay_lattice AMR_REPLAY_LATTICE_ONLY ${NAV2_SMAC_LATTICE_PLUGIN})
add_replay_backend(planner_replay_contract_test AMR_REPLAY_LATTICE_ONLY
  ${NAV2_SMAC_LATTICE_PLUGIN} planner_replay_contract_test.cpp)
enable_testing()
add_test(NAME planner_replay_contract COMMAND planner_replay_contract_test)

install(TARGETS planner_replay_2d planner_replay_lattice DESTINATION lib)
