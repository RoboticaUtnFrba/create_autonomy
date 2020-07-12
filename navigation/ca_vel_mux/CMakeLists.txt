cmake_minimum_required(VERSION 2.8.3)
project(ca_vel_mux)

find_package(catkin REQUIRED COMPONENTS
  yocs_cmd_vel_mux
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
