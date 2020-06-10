cmake_minimum_required(VERSION 2.8.3)
project(planning_plugin_examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_plugin_examples
#  CATKIN_DEPENDS moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface pluginlib roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lerp_planner_example  src/lerp_planner_example.cpp)

target_link_libraries(lerp_planner_example
  ${catkin_LIBRARIES}
)

add_executable(compl_planner_example  src/compl_planner_example.cpp)

target_link_libraries(compl_planner_example
  ${catkin_LIBRARIES}
)

add_executable(try_jacobian_projection  src/try_jacobian_projection.cpp)

target_link_libraries(try_jacobian_projection
  ${catkin_LIBRARIES}
)