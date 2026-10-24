cmake_minimum_required(VERSION 3.16)
project(vgm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_robot_description
    test/test_robot_description.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
