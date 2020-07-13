cmake_minimum_required(VERSION 2.8.3)
project(ProyectoSegmentacion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  move_base
  move_base_msgs
  roscpp
  tf
)

catkin_package(
 CATKIN_DEPENDS
  move_base_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(send_goal src/send_robot_goal.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(send_goal
  ${catkin_LIBRARIES}
)

install(TARGETS send_goal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
