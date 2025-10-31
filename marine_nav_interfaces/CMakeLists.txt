cmake_minimum_required(VERSION 3.8)

project(marine_nav_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  action/ComputeSonarCoveragePath.action
  action/Hover.action
  action/RunTasks.action
  msg/TaskFeedback.msg
  msg/TaskInformation.msg
  msg/TaskInformationList.msg
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()

