cmake_minimum_required(VERSION 2.8.3)
project(ca_tools)

find_package(catkin REQUIRED COMPONENTS
pid
)

catkin_package(
  CATKIN_DEPENDS
    joy_teleop
    geometry_msgs
    std_msgs
    pid
    rospy
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
