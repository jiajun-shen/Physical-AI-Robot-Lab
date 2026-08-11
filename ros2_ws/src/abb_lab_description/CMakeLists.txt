cmake_minimum_required(VERSION 3.16)
project(abb_lab_description)

find_package(ament_cmake REQUIRED)

set(ABB_ASSET_SNAPSHOT
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/abb_irb1200_support")
set(ABB_SUPPORT_SOURCE "${ABB_ASSET_SNAPSHOT}/source/abb_irb1200_support")
set(ABB_RESOURCES_SOURCE "${ABB_ASSET_SNAPSHOT}/source/abb_resources")

foreach(required_path
    "${ABB_ASSET_SNAPSHOT}/LICENSE"
    "${ABB_ASSET_SNAPSHOT}/PROVENANCE.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/abb-irb1200-assets.sha256"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/abb-model-assets.repos"
    "${ABB_SUPPORT_SOURCE}/package.xml"
    "${ABB_SUPPORT_SOURCE}/meshes"
    "${ABB_RESOURCES_SOURCE}/urdf/common_colours.xacro"
    "${ABB_RESOURCES_SOURCE}/urdf/common_materials.xacro")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Required pinned ABB asset is missing: ${required_path}")
  endif()
endforeach()

install(DIRECTORY urdf config third_party
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY "${ABB_SUPPORT_SOURCE}/meshes/"
  DESTINATION share/${PROJECT_NAME}/meshes)

install(FILES
  LICENSE
  NOTICE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(abb_asset_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_irb1200_assets.py"
    TIMEOUT 30)
  ament_add_pytest_test(abb_description_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_description_source.py"
    TIMEOUT 30)
  ament_add_pytest_test(abb_control_description_source_contract
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_abb_control_description_source.py"
    TIMEOUT 30)
endif()

ament_package()
