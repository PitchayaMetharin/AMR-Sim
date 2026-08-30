cmake_minimum_required(VERSION 3.8)
project(amr_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(
  amr-command-watchdog-system SHARED
  src/command_watchdog_system.cpp)
target_include_directories(
  amr-command-watchdog-system PRIVATE include)
target_link_libraries(
  amr-command-watchdog-system
  gz-msgs10::gz-msgs10
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13)
install(TARGETS amr-command-watchdog-system DESTINATION lib)
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks(hooks/gz_sim_system_plugin_path.dsv.in)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_command_watchdog test/test_command_watchdog.cpp)
  target_include_directories(test_command_watchdog PRIVATE include)
  ament_add_pytest_test(simulation_contract_test test/test_simulation_contract.py)
endif()

ament_package()
