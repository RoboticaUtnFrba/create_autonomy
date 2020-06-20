cmake_minimum_required(VERSION 2.8.3)
project(ca_imu)

add_compile_options(-std=c++11)

find_library(RTIMULib libRTIMULib.so REQUIRED)

if(RTIMULib-NOTFOUND)
  message(FATAL_ERROR
    "
    You must install RTIMULib:

    ./create_autonomy/sensors/ca_imu/scripts/install_rtimulib.sh
    "
  )
endif()

find_package(catkin REQUIRED COMPONENTS
  i2c_imu
  imu_filter_madgwick
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
