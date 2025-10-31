cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_bt_task_navigator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(marine_nav_behavior_tree REQUIRED)
find_package(marine_nav_interfaces REQUIRED)
find_package(marine_nav_tasks REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/task_navigator.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${nav2_core_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${nav2_core_LIBRARIES}
  ${marine_nav_interfaces_TARGETS}
  marine_nav_behavior_tree::marine_nav_behavior_tree
  marine_nav_tasks::marine_nav_tasks
  rclcpp::rclcpp
)


pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()

