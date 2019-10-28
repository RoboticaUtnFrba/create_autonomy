cmake_minimum_required(VERSION 2.8.3)
project(ca_actions)

find_package(catkin REQUIRED COMPONENTS
  genmsg 
  actionlib_msgs
  ca_msgs
)

add_action_files(DIRECTORY action FILES LineFollower.action)

generate_messages(DEPENDENCIES actionlib_msgs ca_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime ca_msgs
)
