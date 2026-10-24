cmake_minimum_required(VERSION 3.16)
project(vgm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(configuration_test test/test_configuration.py)
endif()

ament_package()

