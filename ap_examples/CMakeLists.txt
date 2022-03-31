cmake_minimum_required(VERSION 3.0.2)
project(ap_examples)

add_compile_options(-std=c++17)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  affordance_primitives
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib affordance_primitives
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ap_executor src/ap_executor_node.cpp)
target_link_libraries(ap_executor ${catkin_LIBRARIES})
add_dependencies(ap_executor ${catkin_EXPORTED_TARGETS})

add_executable(ap_client src/ap_client_node.cpp)
target_link_libraries(ap_client ${catkin_LIBRARIES})
add_dependencies(ap_client ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ap_client ap_executor 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
