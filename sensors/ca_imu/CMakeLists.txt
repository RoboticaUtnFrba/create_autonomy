cmake_minimum_required(VERSION 2.8.3)
project(ca_imu)

add_compile_options(-std=c++11)

# Automatically install RTIMULib

message(STATUS "Installing RTIMULib...")

execute_process(COMMAND scripts/install_rtimulib.sh
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

find_library(RTIMULib libRTIMULib.so)


find_package(catkin REQUIRED COMPONENTS
  i2c_imu
  rospy
  sensor_msgs
  tf
)

catkin_package(CATKIN_DEPENDS
  rospy
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
