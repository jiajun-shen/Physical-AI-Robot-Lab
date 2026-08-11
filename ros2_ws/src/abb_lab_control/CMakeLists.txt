cmake_minimum_required(VERSION 3.16)
project(abb_lab_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(abb_lab_control_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_lab_control_source.py"
    TIMEOUT 30)
endif()

ament_package()
