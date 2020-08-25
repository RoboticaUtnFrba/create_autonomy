cmake_minimum_required(VERSION 3.5)
project(ca_behavior_tree)

add_compile_options(-std=c++14)

find_package(BehaviorTreeV3)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  move_base_msgs
  roscpp
  roslint
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    behaviortree_cpp_v3
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    move_base_msgs
    roscpp
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/actions/movebase_client.cpp
  src/conditions/is_battery_level_ok.cpp
  src/main.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node BT::behaviortree_cpp_v3 ${catkin_LIBRARIES})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

install(TARGETS
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()
