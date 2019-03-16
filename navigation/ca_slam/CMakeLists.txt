cmake_minimum_required(VERSION 2.8.3)
project(ca_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	# ROS Melodic doesn't support navigation yet
  # slam_gmapping
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
