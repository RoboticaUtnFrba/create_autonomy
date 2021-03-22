cmake_minimum_required(VERSION 3.0.2)
project(ca_mapping)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
