cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_crabbing_path_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_control REQUIRED)
find_package(marine_nav_utilities REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/crabbing_path_follower.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME}
  control_toolbox
  geometry_msgs
  marine_control
  marine_nav_utilities
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rcl_interfaces
  rclcpp
  tf2
  visualization_msgs
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_geometry test/test_path_geometry.cpp)
  if(TARGET test_path_geometry)
    target_include_directories(test_path_geometry PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    ament_target_dependencies(test_path_geometry geometry_msgs)
  endif()

  ament_add_gtest(test_gain_schedule test/test_gain_schedule.cpp)
  if(TARGET test_gain_schedule)
    target_include_directories(test_gain_schedule PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    ament_target_dependencies(test_gain_schedule geometry_msgs)
  endif()

  ament_add_gtest(test_turn_speed_factor test/test_turn_speed_factor.cpp)
  if(TARGET test_turn_speed_factor)
    target_include_directories(test_turn_speed_factor PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    ament_target_dependencies(test_turn_speed_factor geometry_msgs)
  endif()

  # marine_control device-control wiring (unh_marine_autonomy#140 / ADR-0003 —
  # the external marine_control device-control ADR, not this workspace's ADR-0003):
  # exercises the real declareCrabbingDefaultSpeed / declareCrabbingControlParams /
  # bindCrabbingControls helpers against a LifecycleNode + ControlServer (no nav2
  # bring-up). Verifies the twelve controls are advertised with their ranges/groups, a
  # platform _range override is honoured, an in-range change applies, and an
  # out-of-range change is rejected by the FloatingPointRange over the channel.
  ament_add_gtest(test_crabbing_control test/test_crabbing_control.cpp)
  target_link_libraries(test_crabbing_control ${PROJECT_NAME})
  ament_target_dependencies(test_crabbing_control
    marine_control
    marine_control_interfaces
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_package()
