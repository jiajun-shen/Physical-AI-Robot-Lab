cmake_minimum_required(VERSION 3.16)
project(abb_lab_gripper)

find_package(ament_cmake REQUIRED)

# This package intentionally installs only an isolated robot description and
# controller profile. Bringup, spawning, and runtime verification belong to
# their dedicated packages so this profile cannot alter the existing arm-only
# P1/P2 runtime.
install(DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(abb_lab_gripper_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_lab_gripper_source.py"
    TIMEOUT 30)
endif()

ament_package()
