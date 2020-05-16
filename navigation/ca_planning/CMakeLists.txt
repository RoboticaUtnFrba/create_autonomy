cmake_minimum_required(VERSION 3.0.2)
project(ca_planning)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

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
  INCLUDE_DIRS include
  LIBRARIES cuckoo_search_lib cuckoo_search_planner
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

add_subdirectory(include/${PROJECT_NAME}/cuckoo_search)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cuckoo_search_planner src/cuckoo_search.cpp)

add_dependencies(cuckoo_search_planner
        ${cuckoo_search_planner_EXPORTED_TARGETS}
        ${cuckoo_search_lib_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(cuckoo_search_planner
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
  PATTERN ".svn" EXCLUDE
)

install(FILES
  cuckoo_search_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
