cmake_minimum_required(VERSION 3.0.2)
project(ca_planning)

add_compile_options(-std=c++11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)

FIND_PACKAGE(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

set(${JPS_PLUGIN} jps_planner)

catkin_package(
 INCLUDE_DIRS
    include
 LIBRARIES
    # jps
    ${JPS_PLUGIN}
 CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(jps
    src/graph_search.cpp
    src/dmp.cpp
    src/jps_ros.cpp
    src/jps_planner/jps_planner.cpp
    src/jps_planner/graph_search.cpp)
# add_library(dmp_lib src/graph_search.cpp src/dmp.cpp)
# add_library (jps src/jps.cpp src/jps_ros.cpp)
add_dependencies(jps ${jps_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jps ${catkin_LIBRARIES})

add_executable(jps_node src/jps_node.cpp)
target_link_libraries(jps_node jps)

#############
## Install ##
#############

install(TARGETS jps_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS jps
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  ${JPS_PLUGIN}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# https://github.com/ros-planning/navigation/blob/melodic-devel/navfn/src/navtest.cpp

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ca_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
