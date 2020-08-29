cmake_minimum_required(VERSION 3.0.2)
project(ca_planning)

add_compile_options(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)
find_package(jps3d REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  roslint
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES
    jps_ros
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
  ${JPS3D_INCLUDE_DIRS}
)

add_library (jps_ros src/jps_ros.cpp)
target_link_libraries(jps_ros ${catkin_LIBRARIES} ${JPS3D_LIBRARIES})
add_dependencies(jps_ros ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/jps_node.cpp)
target_link_libraries(${PROJECT_NAME}_node jps_ros ${JPS3D_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME jps_node)

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    jps_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

add_executable(${PROJECT_NAME}_test test/test_distance_map_planner_2d.cpp)
target_link_libraries(${PROJECT_NAME}_test ${JPS3D_LIBRARIES} ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_distance_map_planner_2d ${PROJECT_NAME}_test ${CMAKE_SOURCE_DIR}/test/data/corridor.yaml)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-build/header_guard")
roslint_cpp()
