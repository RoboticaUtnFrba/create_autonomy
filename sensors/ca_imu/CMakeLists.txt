cmake_minimum_required(VERSION 2.8.3)
project(ca_imu)

add_compile_options(-std=c++11)

find_library(RTIMULib libRTIMULib.so REQUIRED)

if(RTIMULib-NOTFOUND)
  message(FATAL_ERROR
    "
    You must install RTIMULib:

    . ${PROJECT_SOURCE_DIR}/scripts/install_rtimulib.sh
    "
  )
endif()

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  config
  launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
