cmake_minimum_required(VERSION 3.0.2)
project(ca_planning)

add_compile_options(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

if(YAMLCPP_FOUND)
  message(STATUS "Found yaml-cpp.")
else()
  message(FATAL_ERROR "Could not find yaml-cpp.")
endif()
# FIND_PACKAGE(PkgConfig REQUIRED)
# PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
# if(NOT YAMLCPP_FOUND)
#   # try to use pkg-config
# 	include(FindPkgConfig)
# 	IF(PKG_CONFIG_FOUND)
# 		pkg_check_modules(YAMLCPP QUIET yaml-cpp)
# 	ENDIF(PKG_CONFIG_FOUND)
# endif(NOT YAMLCPP_FOUND)

# IF(YAMLCPP_FOUND)
# 	MESSAGE("-- Found yaml-cpp: ${YAMLCPP_PREFIX} (found version \"${YAMLCPP_VERSION}\")")
# ENDIF(YAMLCPP_FOUND)

###################################
## catkin specific configuration ##
###################################

catkin_package(
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

add_library(${PROJECT_NAME}_lib
    src/dmp.cpp
    src/graph_search.cpp
    src/jps_planner/graph_search.cpp
    src/jps_planner/jps_planner.cpp
    src/jps_ros.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${jps_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME jps_planner)

add_executable(${PROJECT_NAME}_node src/jps_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME jps_node)

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    ${PROJECT_NAME}_lib
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

add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
target_link_libraries(test_distance_map_planner_2d ${PROJECT_NAME}_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/test/data/corridor.yaml)
