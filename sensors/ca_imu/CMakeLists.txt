if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" )
  cmake_minimum_required(VERSION 2.8.3)
  project(ca_imu)
  ## Compile as C++11, supported in ROS Kinetic and newer
  add_compile_options(-std=c++11)

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
  )

  catkin_package(CATKIN_DEPENDS roscpp)
  include_directories(${catkin_INCLUDE_DIRS})

else()
  message( STATUS "Avoid compiling ca_imu because it's not an ARM architecture" )
endif()
