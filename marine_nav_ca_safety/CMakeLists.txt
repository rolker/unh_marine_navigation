cmake_minimum_required(VERSION 3.14.4)

project(marine_nav_ca_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(marine_control REQUIRED)

# Header-only logic + node class; exported as an INTERFACE library.
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

set(ca_safety_deps
  geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  rcl_interfaces
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  marine_control
)

add_executable(ca_safety_node src/ca_safety_node.cpp)
target_include_directories(ca_safety_node PRIVATE include)
ament_target_dependencies(ca_safety_node ${ca_safety_deps})

install(TARGETS ca_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ca_safety_deps})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure logic — no ROS link needed.
  ament_add_gtest(test_ca_safety test/test_ca_safety.cpp)
  target_include_directories(test_ca_safety PRIVATE include)

  ament_add_gtest(test_ca_safety_node test/test_ca_safety_node.cpp)
  target_include_directories(test_ca_safety_node PRIVATE include)
  ament_target_dependencies(test_ca_safety_node ${ca_safety_deps})

  # Cross-process integration: QoS interop (best-effort cloud, reliable cmd_vel),
  # TF transform of a real cloud, and the braking reaction + single output publisher.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_ca_safety_launch.py)

  # marine_control device-control wiring (unh_marine_autonomy#140 / ADR-0003):
  # asserts the node advertises all 15 live tunables on ~/control/state, that an
  # in-range change applies, and that the existing parameter validation (> 0 and
  # the slowdown min <= max ordering) still rejects bad operator input.
  add_launch_test(test/test_ca_safety_control.py)
endif()

ament_package()
