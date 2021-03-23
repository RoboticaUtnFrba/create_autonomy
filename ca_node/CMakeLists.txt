cmake_minimum_required(VERSION 2.8.3)
project(ca_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ca_driver
  ecl_threads
  nodelet
  pluginlib
  roscpp
)

find_package(Boost REQUIRED system thread)

catkin_package(
  LIBRARIES ${PROJECT_NAME}let ca_driver
  CATKIN_DEPENDS
    ca_driver
    ecl_threads
    geometry_msgs
    nodelet
    pluginlib
    roscpp
    rospy
    smach_ros
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES plugins/nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
