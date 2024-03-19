cmake_minimum_required(VERSION 3.8)
project(robot_on_guide_ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(ament_cmake REQUIRED)
find_package(ik_solver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rdyn_core REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
 src/${PROJECT_NAME}/robot_on_guide_ik_solver.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  ik_solver
  tf2_ros)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rdyn_core)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

pluginlib_export_plugin_description_file(ik_solver plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME}
                     HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGES_REQUIRED_HERE})

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}
# ${catkin_LIBRARIES}
#)

ament_package()
