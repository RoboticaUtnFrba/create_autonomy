cmake_minimum_required(VERSION 2.8.3)
project(ca_swarm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  ca_move_base
  geometry_msgs
  move_base
  move_base_msgs
  roscpp
  roslint
  tf
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    amcl
    ca_move_base
    geometry_msgs
    move_base
    move_base_msgs
    roscpp
    tf
    tf2_ros
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN ".svn" EXCLUDE
)

roslint_python()
