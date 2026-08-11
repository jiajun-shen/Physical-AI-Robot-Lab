cmake_minimum_required(VERSION 3.16)
project(abb_lab_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/verify_irb1200_runtime.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(abb_lab_gazebo_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_lab_gazebo_source.py"
    TIMEOUT 30)
endif()

ament_package()
