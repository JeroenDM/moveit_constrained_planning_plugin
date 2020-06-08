cmake_minimum_required(VERSION 3.1)
project(constrained_planning_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  pluginlib
  roscpp
)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS moveit_core moveit_msgs pluginlib roscpp
  DEPENDS OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/compl_planning_context.cpp
  src/compl_interface.cpp
  src/constraint.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_library(
  moveit_constrained_planning_plugin
  src/compl_planner_manager.cpp
)

target_link_libraries(
  moveit_constrained_planning_plugin
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)