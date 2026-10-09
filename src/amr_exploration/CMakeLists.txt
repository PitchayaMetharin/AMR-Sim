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
  ament_add_pytest_test(frontier_approach_test test/test_frontier_approach.py)
  ament_add_pytest_test(frontier_contract_test test/test_frontier_contract.py)
  ament_add_pytest_test(frontier_blockage_terminal_contract_test test/test_frontier_blockage_terminal_contract.py)
  ament_add_pytest_test(frontier_evidence_test test/test_frontier_evidence.py)
  ament_add_pytest_test(frontier_goal_clearance_test test/test_frontier_goal_clearance.py)
  ament_add_pytest_test(frontier_goal_heading_test test/test_frontier_goal_heading.py)
  ament_add_pytest_test(frontier_route_margin_test test/test_frontier_route_margin.py)
  ament_add_pytest_test(frontier_lifecycle_test test/test_frontier_lifecycle.py)
  ament_add_pytest_test(frontier_localization_recovery_test test/test_frontier_localization_recovery.py)
  ament_add_pytest_test(frontier_patrol_test test/test_frontier_patrol.py)
  ament_add_pytest_test(frontier_planning_speed_test test/test_frontier_planning_speed.py)
endif()

ament_package()
