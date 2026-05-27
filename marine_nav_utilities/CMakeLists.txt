cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/utilities.cpp
  src/costmap_window.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries( ${PROJECT_NAME}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(costmap_window_node src/costmap_window_node.cpp)
target_link_libraries(costmap_window_node
  ${PROJECT_NAME}
  ${nav_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS costmap_window_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rcl_interfaces
  tf2
  tf2_geometry_msgs
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_costmap_window test/test_costmap_window.cpp)
  target_link_libraries(test_costmap_window
    ${PROJECT_NAME}
    ${nav_msgs_TARGETS}
  )

  ament_add_gtest(test_costmap_window_node test/test_costmap_window_node.cpp)
  target_link_libraries(test_costmap_window_node
    ${PROJECT_NAME}
    ${nav_msgs_TARGETS}
    rclcpp::rclcpp
  )
endif()

ament_package()

