cmake_minimum_required(VERSION 3.8)
project(amr_exploration)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/frontier_algorithm.py
  scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(frontier_algorithm_test test/test_frontier_algorithm.py)
  ament_add_pytest_test(frontier_contract_test test/test_frontier_contract.py)
endif()

ament_package()
