cmake_minimum_required(VERSION 3.8)
project(amr_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(slam_contract_test test/test_slam_contract.py)
endif()

ament_package()
