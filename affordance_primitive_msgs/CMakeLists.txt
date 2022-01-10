cmake_minimum_required(VERSION 3.0.2)
project(affordance_primitive_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ScrewStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AffordancePrimitive.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
