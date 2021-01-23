cmake_minimum_required(VERSION 2.8.3)
project(ca_move_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf2
  tf2_geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  actionlib
  move_base_msgs
  roscpp
  tf2
  tf2_geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(send_move_base_goal src/send_robot_goal.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(send_move_base_goal
  ${catkin_LIBRARIES}
)

install(TARGETS send_move_base_goal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
