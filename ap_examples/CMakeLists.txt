cmake_minimum_required(VERSION 3.5)
project(ap_examples)

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
find_package(affordance_primitives REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    affordance_primitives
    rclcpp
)

include_directories(
  include
)

add_library(ap_example_plugins SHARED
  src/example_parameter_manager.cpp
  src/example_task_estimator.cpp
)
target_include_directories(ap_example_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ap_example_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS} pluginlib)

pluginlib_export_plugin_description_file(affordance_primitives ap_example_plugins.xml)

add_executable(ap_executor src/ap_executor_node.cpp)
ament_target_dependencies(ap_executor ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(ap_client src/ap_client_node.cpp)
ament_target_dependencies(ap_client ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ap_example_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ap_client
  ap_executor
  DESTINATION lib/${PROJECT_NAME})

install(FILES ap_example_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_libraries(ap_example_plugins)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
