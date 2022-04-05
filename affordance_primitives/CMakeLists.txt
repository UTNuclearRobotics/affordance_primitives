cmake_minimum_required(VERSION 3.5)
project(affordance_primitives)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # Enable warnings
  add_compile_options(-Wall -Wextra
    -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual
    -Wno-unused-parameter -Wno-unused-function)
else()
  # Defaults for Microsoft C++ compiler
  add_compile_options(/W3 /wd4251 /wd4068 /wd4275)

  # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(affordance_primitive_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    affordance_primitive_msgs
    geometry_msgs
    pluginlib
    tf2_eigen
    tf2_ros
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/ap_executor/ap_executor.hpp
  include/${PROJECT_NAME}/configs_interface/empty_parameter_manager.hpp
  include/${PROJECT_NAME}/configs_interface/parameter_manager.hpp
  include/${PROJECT_NAME}/screw_model/screw_axis.hpp
  include/${PROJECT_NAME}/screw_model/screw_execution.hpp
  include/${PROJECT_NAME}/task_estimator/kinematic_task_estimator.hpp
  include/${PROJECT_NAME}/task_estimator/task_estimator.hpp
  include/${PROJECT_NAME}/task_monitor/task_monitor.hpp
  src/ap_executor/ap_executor.cpp
  src/configs_interface/empty_parameter_manager.cpp
  src/screw_model/affordance_utils.cpp
  src/screw_model/screw_axis.cpp
  src/screw_model/screw_execution.cpp
  src/task_estimator/kinematic_task_estimator.cpp
  src/task_monitor/task_monitor.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})







#############
## TESTING ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   catkin_add_gtest(test_affordance_utils test/test_affordance_utils.cpp)
#   target_link_libraries(test_affordance_utils ${catkin_LIBRARIES} ${PROJECT_NAME})

#   catkin_add_gtest(test_screw_axis test/test_screw_axis.cpp)
#   target_link_libraries(test_screw_axis ${catkin_LIBRARIES} ${PROJECT_NAME})

#   add_rostest_gtest(test_screw_execution test/test_screw_execution.test test/test_screw_execution.cpp)
#   target_link_libraries(test_screw_execution ${catkin_LIBRARIES} ${PROJECT_NAME})

#   add_rostest_gtest(test_parameter_manager test/test_parameter_manager.test test/test_parameter_manager.cpp)
#   target_link_libraries(test_parameter_manager ${catkin_LIBRARIES} ${PROJECT_NAME})

#   add_rostest_gtest(test_task_estimator test/test_task_estimator.test test/test_task_estimator.cpp)
#   target_link_libraries(test_task_estimator ${catkin_LIBRARIES} ${PROJECT_NAME})

#   add_rostest_gtest(test_task_monitor test/test_task_monitor.test test/test_task_monitor.cpp)
#   target_link_libraries(test_task_monitor ${catkin_LIBRARIES} ${PROJECT_NAME})

#   add_rostest_gtest(test_ap_executor test/test_ap_executor.test test/test_ap_executor.cpp)
#   target_link_libraries(test_ap_executor ${catkin_LIBRARIES} ${PROJECT_NAME})
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
