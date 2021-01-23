cmake_minimum_required(VERSION 2.8.3)
project(ca_bringup)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

#############
## Install ##
#############

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
