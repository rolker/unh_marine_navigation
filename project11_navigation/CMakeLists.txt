cmake_minimum_required(VERSION 3.5.1)
project(project11_navigation)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp 4 REQUIRED)
find_package(dubins_curves REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(marine_nav_behavior_tree REQUIRED)
find_package(marine_nav_interfaces REQUIRED)
find_package(marine_nav_tasks REQUIRED)
find_package(marine_nav_utilities REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(project11 REQUIRED)
find_package(project11_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/context.cpp
  src/environment.cpp
  src/navigator_settings.cpp
  src/occupancy_grid.cpp
  src/platform.cpp
  src/robot.cpp
  src/robot_capabilities.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${grid_map_ros_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${action_msgs_TARGETS}
  marine_nav_behavior_tree::marine_nav_behavior_tree
  project11::project11
  ${project11_nav_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  ${tf2_geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  dubins_curves::dubins_curves
  ${grid_map_ros_LIBRARIES}
  marine_nav_utilities::marine_nav_utilities
  nav2_util::nav2_util_core
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
  rclcpp::rclcpp
  yaml-cpp::yaml-cpp
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS project11_navigation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# navigator

add_executable(navigator
  src/main.cpp
  src/navigator.cpp
)

target_include_directories(navigator PRIVATE ${GENERATED_DIR})

target_link_libraries(navigator
  ament_index_cpp::ament_index_cpp
  project11::project11
  project11_navigation::project11_navigation
)

ament_target_dependencies(navigator
  behaviortree_cpp
  marine_nav_interfaces
  marine_nav_tasks
  nav2_util
  project11_nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
)



add_executable(occupancy_vector_map_from_geo src/tools/occupancy_vector_map_from_geo.cpp)
target_link_libraries(occupancy_vector_map_from_geo
  project11::project11
  ${project11_nav_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(occupancy_grid_from_vector_map src/tools/occupancy_grid_from_vector_map.cpp)
target_include_directories(occupancy_grid_from_vector_map PRIVATE ${grid_map_ros_INCLUDE_DIRS})
target_link_libraries(occupancy_grid_from_vector_map
  ${grid_map_ros_LIBRARIES}
  project11::project11
  ${project11_nav_msgs_TARGETS}
  rclcpp::rclcpp
)


install(TARGETS navigator occupancy_vector_map_from_geo occupancy_grid_from_vector_map
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  behaviortree_cpp
  dubins_curves
  marine_nav_behavior_tree
  nav_msgs
  project11
  project11_msgs
  project11_nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  yaml_cpp_vendor
)

ament_package()

