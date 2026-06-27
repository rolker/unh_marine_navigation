cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_avoidance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_control REQUIRED)
find_package(marine_nav_utilities REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/avoidance_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${PROJECT_NAME}
  marine_nav_utilities::marine_nav_utilities
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  marine_control
  marine_nav_utilities
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # The node-free planning core (nearest-station + anchor-behind active range +
  # cost-matrix + solve) is unit-tested as planCorridorOffsets in
  # marine_nav_utilities (test_corridor_solver). The controller's remaining code
  # is ROS plumbing (configure/lifecycle/delegate) + costmap sampling, validated
  # in sim + on-water per the #59 acceptance criteria.

  # marine_control device-control wiring (unh_marine_autonomy#140 / ADR-0003):
  # exercises the real declareAvoidanceControlParams / bindAvoidanceControls
  # helpers against a LifecycleNode + ControlServer (no nav2 bring-up). Verifies
  # the 10 tunables are advertised with their ranges/groups, a platform _range
  # override is honoured, an in-range change applies, and an out-of-range change
  # is rejected by the FloatingPointRange over the marine_control channel.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_avoidance_control test/test_avoidance_control.cpp)
  target_link_libraries(test_avoidance_control ${PROJECT_NAME})
  ament_target_dependencies(test_avoidance_control
    marine_control
    marine_control_interfaces
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_package()
