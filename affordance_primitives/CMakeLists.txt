cmake_minimum_required(VERSION 3.0.2)
project(affordance_primitives)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Winline)

find_package(catkin REQUIRED COMPONENTS
  affordance_primitive_msgs
  geometry_msgs
  pluginlib
  tf2_eigen
  tf2_ros
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    affordance_primitive_msgs
    geometry_msgs
    pluginlib
    tf2_eigen
    tf2_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    affordance_primitive_msgs
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/configs_interface/empty_parameter_manager.hpp
  include/${PROJECT_NAME}/configs_interface/parameter_manager.hpp
  include/${PROJECT_NAME}/screw_model/screw_axis.hpp
  include/${PROJECT_NAME}/screw_model/screw_conversions.hpp
  include/${PROJECT_NAME}/screw_model/screw_execution.hpp
  include/${PROJECT_NAME}/task_estimator/kinematic_task_estimator.hpp
  include/${PROJECT_NAME}/task_estimator/task_estimator.hpp
  src/configs_interface/empty_parameter_manager.cpp
  src/screw_model/affordance_utils.cpp
  src/screw_model/screw_axis.cpp
  src/screw_model/screw_execution.cpp
  src/task_estimator/kinematic_task_estimator.cpp
)
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

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## TESTING ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_affordance_utils test/test_affordance_utils.cpp)
  target_link_libraries(test_affordance_utils ${catkin_LIBRARIES} ${PROJECT_NAME})

  catkin_add_gtest(test_screw_axis test/test_screw_axis.cpp)
  target_link_libraries(test_screw_axis ${catkin_LIBRARIES} ${PROJECT_NAME})

  add_rostest_gtest(test_screw_execution test/test_screw_execution.test test/test_screw_execution.cpp)
  target_link_libraries(test_screw_execution ${catkin_LIBRARIES} ${PROJECT_NAME})

  add_rostest_gtest(test_parameter_manager test/test_parameter_manager.test test/test_parameter_manager.cpp)
  target_link_libraries(test_parameter_manager ${catkin_LIBRARIES} ${PROJECT_NAME})

  add_rostest_gtest(test_task_estimator test/test_task_estimator.test test/test_task_estimator.cpp)
  target_link_libraries(test_task_estimator ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
