cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(marine_nav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(marine_nav_hover_behavior SHARED
  src/hover.cpp
)

target_include_directories(marine_nav_hover_behavior
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


target_compile_definitions(marine_nav_hover_behavior PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(marine_nav_hover_behavior
  nav2_behaviors
  pluginlib
  marine_nav_interfaces
  rclcpp
  rclcpp_lifecycle
  visualization_msgs
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS marine_nav_hover_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hover_heading
    test/test_hover_heading.cpp
  )
  if(TARGET test_hover_heading)
    target_include_directories(test_hover_heading PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
  endif()

  ament_add_gtest(test_hover_speed
    test/test_hover_speed.cpp
  )
  if(TARGET test_hover_speed)
    target_include_directories(test_hover_speed PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )
  endif()
endif()

ament_package()
