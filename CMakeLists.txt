cmake_minimum_required(VERSION 3.0.2)
project(affordance_primitives)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Winline)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_eigen
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    geometry_msgs
    tf2_eigen
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/screw_motion.cpp
  src/affordance_utils.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## TESTING ##
#############

if(CATKIN_ENABLE_TESTING)
  # find_package(rostest REQUIRED)

  catkin_add_gtest(test_affordance_utils test/test_affordance_utils.cpp)
  target_link_libraries(test_affordance_utils ${catkin_LIBRARIES} ${PROJECT_NAME})

  catkin_add_gtest(test_screw_axis test/test_screw_axis.cpp)
  target_link_libraries(test_screw_axis ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
