cmake_minimum_required(VERSION 2.8.3)
project(robot_servicio)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base
  move_base_msgs
  rospy
  std_msgs
  tf
  amcl
  octomap_server
)

catkin_package(
 CATKIN_DEPENDS
  move_base_msgs
)

include_directories(${catkin_INCLUDE_DIRS})