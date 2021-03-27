cmake_minimum_required(VERSION 3.0.2)
project(franka_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
   controller_interface
   controller_manager
   dynamic_reconfigure
   eigen_conversions
   franka_hw
   geometry_msgs
   hardware_interface
   tf
   tf_conversions
   message_generation
   moveit_msgs
   moveit_core
   moveit_visual_tools
   moveit_ros_planning_interface
   pluginlib
   realtime_tools
   roscpp
   rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)



generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES franka_controllers
   CATKIN_DEPENDS
      controller_interface
      dynamic_reconfigure
      eigen_conversions
      franka_hw
      geometry_msgs
      hardware_interface
      tf
      tf_conversions
      message_runtime
      moveit_msgs 
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      pluginlib
      realtime_tools
      roscpp
   DEPENDS Franka
)

add_library(franka_controllers
   src/franka_state_controller.cpp
   src/joint_velocity_controller.cpp
   src/joint_position_controller.cpp
   src/cartesian_velocity_controller.cpp
   src/cartesian_impedance_controller.cpp
)


add_dependencies(franka_controllers
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_controllers PUBLIC
   ${Franka_LIBRARIES}
   ${catkin_LIBRARIES}
)

target_include_directories(franka_controllers SYSTEM PUBLIC
   ${Franka_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)
target_include_directories(franka_controllers PUBLIC
   include
   ${catkin_INCLUDE_DIRS}
)


## franka_control_node
add_executable(franka_control_node
  src/franka_control_node.cpp
)

add_dependencies(franka_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(franka_control_node
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(franka_control_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



# Table collision object publisher

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
add_executable(robot_base_publisher src/collision_object/robot_base_publisher.cpp)
target_link_libraries(robot_base_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})





## Installation
install(TARGETS franka_controllers
                franka_control_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_controllers_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
   RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
   file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
   file(GLOB_RECURSE HEADERS
   ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
   ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
   add_format_target(franka_controllers FILES ${SOURCES} ${HEADERS})
   add_tidy_target(franka_controllers
      FILES ${SOURCES}
      DEPENDS franka_controllers franka_control_node
  )
endif()
