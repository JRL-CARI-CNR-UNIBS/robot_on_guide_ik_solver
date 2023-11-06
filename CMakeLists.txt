cmake_minimum_required(VERSION 3.0.2)
project(robot_on_guide_ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  ik_solver
  roscpp
  rosdyn_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_on_guide_ik_solver
  CATKIN_DEPENDS ik_solver roscpp rosdyn_core
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
 src/${PROJECT_NAME}/robot_on_guide_ik_solver.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
)
