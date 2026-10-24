cmake_minimum_required(VERSION 3.8)
project(vgm_motion)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vgm_interfaces REQUIRED)

add_library(${PROJECT_NAME}_policy src/motion_policy.cpp)
target_compile_features(${PROJECT_NAME}_policy PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(motion_server
  src/main.cpp
  src/motion_server.cpp
)
target_compile_features(motion_server PUBLIC cxx_std_17)
target_include_directories(motion_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(motion_server ${PROJECT_NAME}_policy)
ament_target_dependencies(motion_server
  builtin_interfaces
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  vgm_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_policy motion_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_motion_policy test/test_motion_policy.cpp)
  target_link_libraries(test_motion_policy ${PROJECT_NAME}_policy)
  target_compile_features(test_motion_policy PUBLIC cxx_std_17)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_policy)
ament_package()
