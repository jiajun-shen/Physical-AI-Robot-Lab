cmake_minimum_required(VERSION 3.16)
project(physical_ai_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetection.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/ObjectPose.msg"
  "msg/ObjectPoseArray.msg"
  "msg/SpeechCommand.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(physical_ai_interfaces_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_interfaces_contract.py"
    TIMEOUT 30)
endif()

ament_package()
