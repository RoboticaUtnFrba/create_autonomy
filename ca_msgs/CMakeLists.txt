cmake_minimum_required(VERSION 2.8.3)
project(ca_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Bumper.msg
  ChargingState.msg
  Cliff.msg
  DefineSong.msg
  Mode.msg
  Overcurrent.msg
  PlaySong.msg
  Wheeldrop.msg
  LineFollowerMetrics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)