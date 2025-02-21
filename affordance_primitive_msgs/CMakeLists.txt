cmake_minimum_required(VERSION 3.5)
project(affordance_primitive_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/APRobotParameter.msg"
  "msg/AffordanceTrajectory.msg"
  "msg/AffordanceWaypoint.msg"
  "msg/CartesianBool.msg"
  "msg/CartesianFloat.msg"
  "msg/ScrewStamped.msg"
)

set(action_files
  "action/AffordancePrimitive.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
