cmake_minimum_required(VERSION 3.0.2)
project(ca_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    cuckoo_search_lib
    cuckoo_search_planner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##############################################
# CUCKOO SEARCH LIB

set(CS_LIB_PATH lib/cuckoo_search_lib)
set(CS_LIB_NAME cuckoo_search_lib)

include_directories(${CS_LIB_PATH}/include ${catkin_INCLUDE_DIRS})

# Find source files
file(GLOB SOURCES ${CS_LIB_PATH}/src/*.cpp)

# Create shared library
add_library(${CS_LIB_NAME} SHARED ${SOURCES})

add_dependencies(${CS_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${CS_LIB_NAME} ${catkin_LIBRARIES} -ltbb)

install(TARGETS ${CS_LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY ${CS_LIB_PATH}/include/${CS_LIB_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##############################################

add_library(cuckoo_search_planner src/cuckoo_search.cpp)

add_dependencies(cuckoo_search_planner
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(cuckoo_search_planner
  cuckoo_search_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS cuckoo_search_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  cuckoo_search_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
