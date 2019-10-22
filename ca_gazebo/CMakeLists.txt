if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" )
  message( STATUS "Avoid compiling ca_gazebo because of the ARM architecture" )
else()
  cmake_minimum_required(VERSION 2.8.3)
	set(CMAKE_CXX_FLAGS "-std=c++11")
	project(ca_gazebo)

	find_package(gazebo REQUIRED)

  find_package(catkin REQUIRED COMPONENTS
				gazebo_ros
				gazebo_plugins
				roscpp
				ca_msgs
				apriltag_ros
				tf
	)

  catkin_package(
		INCLUDE_DIRS include
		LIBRARIES create_bumper_plugin
		CATKIN DEPENDS
			gazebo_ros
			gazebo_plugins
			roscpp
			ca_msgs
			apriltag_ros
			tf
	)

	link_directories(${GAZEBO_LIBRARY_DIRS})
  include_directories(include
    ${catkin_INCLUDE_DIRS}
		${GAZEBO_INCLUDE_DIRS}
  )

	add_library(create_bumper_plugin src/create_bumper_plugin.cpp)
	add_dependencies(create_bumper_plugin ${catkin_EXPORTED_TARGETS})
	target_link_libraries(create_bumper_plugin
				${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

  install(DIRECTORY launch worlds models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  message(STATUS "ca_gazebo compiled")
endif()
