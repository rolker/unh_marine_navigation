cmake_minimum_required(VERSION 3.5.1)
project(project11_navigation)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp 4 REQUIRED)
find_package(dubins_curves REQUIRED)
find_package(grid_map_ros REQUIRED)
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

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/context.cpp
  src/environment.cpp
  src/navigator_settings.cpp
  src/occupancy_grid.cpp
  src/platform.cpp
  src/robot.cpp
  src/robot_capabilities.cpp
  src/task.cpp
  src/task_list.cpp
  src/utilities.cpp
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
  project11::project11
  ${project11_nav_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  ${tf2_geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  dubins_curves::dubins_curves
  ${grid_map_ros_LIBRARIES}
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


# behavior tree plugins
set(BT_PLUGIN_NODES
  plugins/action/baxevani_controller.cpp
  plugins/action/clear_path.cpp
  plugins/action/crabbing_path_follower.cpp
  plugins/action/debug_blackboard.cpp
  plugins/action/follow_path_cancel_node.cpp
  plugins/action/generate_plan.cpp
  plugins/action/get_task_data_double.cpp
  plugins/action/get_task_data_string.cpp
  plugins/action/get_sub_tasks.cpp
  plugins/action/hover_action.cpp
  plugins/action/hover_cancel_node.cpp
  plugins/action/multibeam_coverage_action.cpp
  plugins/action/predict_stopping_pose.cpp
  plugins/action/pose_vector_to_path.cpp
  plugins/action/set_pose_from_task.cpp
  plugins/action/set_task_done.cpp
  plugins/action/set_trajectory_from_task.cpp
  plugins/action/task_list_updater.cpp
  plugins/action/update_current_segment.cpp
  plugins/action/update_current_task.cpp
  plugins/action/visualize_trajectory.cpp
  plugins/condition/all_tasks_done.cpp
  plugins/condition/goal_reached.cpp
  plugins/condition/path_empty.cpp
  plugins/condition/plan_needed.cpp
  plugins/condition/task_updated.cpp
)

add_library(project11_navigation_bt_plugins SHARED
  ${BT_PLUGIN_NODES}
  src/bt_register_nodes.cpp
)
list(APPEND plugin_libs project11_navigation_bt_plugins)

target_include_directories(project11_navigation_bt_plugins
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(project11_navigation_bt_plugins
  project11::project11
  project11_navigation
)

ament_target_dependencies(project11_navigation_bt_plugins
  behaviortree_cpp
  dubins_curves
  grid_map_ros
  nav_msgs
  nav2_behavior_tree
  nav2_util
  project11_msgs
  project11_nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

target_compile_definitions(project11_navigation_bt_plugins PRIVATE BT_PLUGIN_EXPORT)


# from nav2_behavior_tree
# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_project11_navigation_tree_nodes_xml src/generate_project11_navigation_tree_nodes_xml.cpp)
ament_target_dependencies(generate_project11_navigation_tree_nodes_xml 
  behaviortree_cpp
  nav2_util
)
# allow generate_nav2_tree_nodes_xml to find plugins_list.hpp
target_include_directories(generate_project11_navigation_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_project11_navigation_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})


install(FILES project11_navigation_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})



# behaviors

set(hover_plugin project11_navigation_hover_behavior_plugin)

add_library(${hover_plugin} SHARED
  plugins/behaviors/hover.cpp
)

target_include_directories(${hover_plugin}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_compile_definitions(${hover_plugin} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${hover_plugin}
  nav2_behaviors
  pluginlib
  project11_nav_msgs
  rclcpp
  rclcpp_lifecycle
  visualization_msgs
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS ${hover_plugin}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# controllers

set(path_follower_plugin project11_navigation_crabbing_path_follower_controller_plugin)

add_library(${path_follower_plugin} SHARED
  plugins/controllers/crabbing_path_follower.cpp
)

target_include_directories(${path_follower_plugin}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_compile_definitions(${path_follower_plugin} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${path_follower_plugin}
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  project11
  project11_msgs
  rclcpp
  tf2
)

pluginlib_export_plugin_description_file(nav2_core crabbing_path_follower_plugin.xml)

install(TARGETS ${path_follower_plugin}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES crabbing_path_follower_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


# navigator

add_executable(navigator
  src/bt_types.cpp
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
  nav2_util
  project11_nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
)


# task navigator

add_library(task_navigator SHARED
  src/bt_types.cpp
  src/task_navigator.cpp
)

target_include_directories(task_navigator
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(task_navigator
  PUBLIC
  ${nav2_core_INCLUDE_DIRS}
)

target_link_libraries(task_navigator PUBLIC
  ${nav2_core_LIBRARIES}
  project11::project11
  ${project11_nav_msgs_TARGETS}
  
)


pluginlib_export_plugin_description_file(nav2_core navigator_plugin.xml)

install(
  TARGETS project11_navigation_bt_plugins task_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})



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

