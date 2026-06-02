cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_behavior_tree)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp 4 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_nav_interfaces REQUIRED)
find_package(marine_nav_tasks REQUIRED)
find_package(marine_nav_utilities REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/bt_types.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  geometry_msgs
  marine_nav_tasks
  nav2_util
  std_msgs
  tf2_ros
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  geometry_msgs
  marine_nav_tasks
  nav2_util
  std_msgs
  tf2_ros
)

install (
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_bt_plugins SHARED
  src/bt_register_nodes.cpp
  src/plugins/action/adjust_path_for_obstacles.cpp
  src/plugins/action/add_sub_task.cpp
  src/plugins/action/clear_path.cpp
  src/plugins/action/fix_path_orientations.cpp
  src/plugins/action/follow_path_cancel_node.cpp
  src/plugins/action/get_sub_path.cpp
  src/plugins/action/get_sub_tasks.cpp
  src/plugins/action/get_task_data_double.cpp
  src/plugins/action/get_task_data_string.cpp
  src/plugins/action/hover_action.cpp
  src/plugins/action/hover_cancel_node.cpp
  src/plugins/action/path_to_pose_vector.cpp
  src/plugins/action/predict_stopping_pose.cpp
  src/plugins/action/set_controller_speed.cpp
  src/plugins/action/set_path_from_task.cpp
  src/plugins/action/set_polygon_from_task.cpp
  src/plugins/action/set_task_done.cpp
  src/plugins/action/set_task_failed.cpp
  src/plugins/action/sonar_coverage_action.cpp
  src/plugins/action/sonar_coverage_cancel_node.cpp
  src/plugins/action/update_current_task.cpp
  src/plugins/condition/all_tasks_done.cpp
  src/plugins/condition/has_sub_tasks.cpp
  src/plugins/condition/path_empty.cpp
  src/plugins/condition/robot_on_path.cpp
  src/plugins/decorator/restart_on_task_change.cpp
)

list(APPEND plugin_libs ${PROJECT_NAME}_bt_plugins)

target_include_directories(${PROJECT_NAME}_bt_plugins
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_bt_plugins
  marine_nav_tasks::marine_nav_tasks
)

ament_target_dependencies(${PROJECT_NAME}_bt_plugins
  behaviortree_cpp
  geometry_msgs
  marine_nav_interfaces
  marine_nav_utilities
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  nav_msgs
  rcl_interfaces
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

target_compile_definitions(${PROJECT_NAME}_bt_plugins PRIVATE BT_PLUGIN_EXPORT)

install(
  TARGETS ${PROJECT_NAME}_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# from nav2_behavior_tree
# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)

configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

add_executable(generate_${PROJECT_NAME}_nodes_xml
  src/generate_${PROJECT_NAME}_nodes_xml.cpp
)

# allow tree nodes xml generator to find plugins_list.hpp
target_include_directories(generate_${PROJECT_NAME}_nodes_xml PRIVATE ${GENERATED_DIR}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)


target_link_libraries(generate_${PROJECT_NAME}_nodes_xml
  ${PROJECT_NAME}_bt_plugins
  ${marine_nav_interfaces_TARGETS}
)

ament_target_dependencies(generate_${PROJECT_NAME}_nodes_xml 
  behaviortree_cpp
  nav2_util
)

install(TARGETS generate_${PROJECT_NAME}_nodes_xml DESTINATION lib/${PROJECT_NAME})

add_custom_command(TARGET generate_${PROJECT_NAME}_nodes_xml POST_BUILD
  COMMAND generate_${PROJECT_NAME}_nodes_xml
  BYPRODUCTS ${GENERATED_DIR}/marine_nav_behavior_tree_nodes.xml
  WORKING_DIRECTORY ${GENERATED_DIR}
  COMMENT "Generating marine_nav_behavior_tree_nodes.xml"
)


install(FILES ${GENERATED_DIR}/marine_nav_behavior_tree_nodes.xml DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_set_controller_speed_resolve
    test/test_set_controller_speed_resolve.cpp
  )
  if(TARGET test_set_controller_speed_resolve)
    target_include_directories(test_set_controller_speed_resolve PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_set_controller_speed_resolve
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_set_controller_speed_resolve
      behaviortree_cpp
      rclcpp
    )
  endif()

  ament_add_gtest(test_set_task_failed
    test/test_set_task_failed.cpp
  )
  if(TARGET test_set_task_failed)
    target_include_directories(test_set_task_failed PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_set_task_failed
      ${PROJECT_NAME}_bt_plugins
      marine_nav_tasks::marine_nav_tasks
    )
    ament_target_dependencies(test_set_task_failed
      behaviortree_cpp
      marine_nav_interfaces
      rclcpp
    )
  endif()

  ament_add_gtest(test_dispatch_routing
    test/test_dispatch_routing.cpp
  )
  if(TARGET test_dispatch_routing)
    target_include_directories(test_dispatch_routing PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_dispatch_routing
      ${PROJECT_NAME}_bt_plugins
      marine_nav_tasks::marine_nav_tasks
    )
    ament_target_dependencies(test_dispatch_routing
      behaviortree_cpp
      marine_nav_interfaces
      rclcpp
    )
  endif()

  ament_add_gtest(test_predict_stopping_pose
    test/test_predict_stopping_pose.cpp
  )
  if(TARGET test_predict_stopping_pose)
    target_include_directories(test_predict_stopping_pose PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_predict_stopping_pose
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_predict_stopping_pose
      behaviortree_cpp
      geometry_msgs
      tf2
      tf2_geometry_msgs
    )
  endif()

  ament_add_gtest(test_set_path_from_task
    test/test_set_path_from_task.cpp
  )
  if(TARGET test_set_path_from_task)
    target_include_directories(test_set_path_from_task PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_set_path_from_task
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_set_path_from_task
      behaviortree_cpp
      geometry_msgs
      nav_msgs
    )
  endif()

  ament_add_gtest(test_get_sub_path
    test/test_get_sub_path.cpp
  )
  if(TARGET test_get_sub_path)
    target_include_directories(test_get_sub_path PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_get_sub_path
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_get_sub_path
      behaviortree_cpp
      geometry_msgs
      nav_msgs
    )
  endif()

  ament_add_gtest(test_hover_reentry
    test/test_hover_reentry.cpp
  )
  if(TARGET test_hover_reentry)
    target_include_directories(test_hover_reentry PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_hover_reentry
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_hover_reentry
      behaviortree_cpp
      rclcpp
    )
  endif()

  ament_add_gtest(test_adjust_path_for_obstacles
    test/test_adjust_path_for_obstacles.cpp
  )
  if(TARGET test_adjust_path_for_obstacles)
    target_include_directories(test_adjust_path_for_obstacles PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_adjust_path_for_obstacles
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_adjust_path_for_obstacles
      behaviortree_cpp
      geometry_msgs
      nav2_msgs
      nav_msgs
    )
  endif()

  ament_add_gtest(test_goto_reentry
    test/test_goto_reentry.cpp
  )
  if(TARGET test_goto_reentry)
    target_include_directories(test_goto_reentry PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_goto_reentry
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_goto_reentry
      behaviortree_cpp
      rclcpp
    )
  endif()

  ament_add_gtest(test_robot_on_path
    test/test_robot_on_path.cpp
  )
  if(TARGET test_robot_on_path)
    target_include_directories(test_robot_on_path PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
    target_link_libraries(test_robot_on_path
      ${PROJECT_NAME}_bt_plugins
    )
    ament_target_dependencies(test_robot_on_path
      behaviortree_cpp
      geometry_msgs
      nav_msgs
    )
  endif()
endif()

ament_package()


