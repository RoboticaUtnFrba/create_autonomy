cmake_minimum_required(VERSION 2.8.3)
project(ca_camera)

find_package(catkin REQUIRED COMPONENTS
  astra_camera
  camera_calibration
  depth_image_proc
  image_proc
  openni2_launch
  realsense2_camera
	rplidar_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	USE_SOURCE_PERMISSIONS
)
