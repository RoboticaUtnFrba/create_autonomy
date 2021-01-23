#catkin_lint: ignore plugin_missing_install
cmake_minimum_required(VERSION 2.8.3)
project(ca_bumper2pc)
find_package(catkin REQUIRED COMPONENTS
  ca_msgs
  nodelet
  pluginlib
  roscpp
  roslint
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS
        ca_msgs
        nodelet
        pluginlib
        roscpp
        sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES plugins/nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)

install(DIRECTORY launch plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()
