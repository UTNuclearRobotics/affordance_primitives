find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_screw_axis
  test_screw_axis.cpp)
target_link_libraries(test_screw_axis ${PROJECT_NAME})
target_include_directories(test_screw_axis
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)

ament_add_gtest(test_affordance_utils
  test_affordance_utils.cpp)
target_link_libraries(test_affordance_utils ${PROJECT_NAME})
target_include_directories(test_affordance_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
