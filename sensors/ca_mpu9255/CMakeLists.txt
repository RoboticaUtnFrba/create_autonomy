if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" )
  cmake_minimum_required(VERSION 2.8.3)
  project(ca_mpu9255)

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    rospy
    sensor_msgs
    std_msgs
    tf
  )

  catkin_python_setup()
  catkin_package()
  include_directories(${catkin_INCLUDE_DIRS})

	# MPU9255 node
	add_executable(mpu9255_node src/mpu9255_node.cpp)
	target_link_libraries(mpu9255_node ${catkin_LIBRARIES})
	target_link_libraries (mpu9255_node wiringPi)

  # MPU9255 offset node
	add_executable(mpu9255_offset src/offsetIMU.cpp)
	target_link_libraries(mpu9255_offset ${catkin_LIBRARIES})
	target_link_libraries (mpu9255_offset wiringPi)

else()
  message( STATUS "Avoid compiling ca_mpu6050 because it's not an ARM architecture" )
endif()
