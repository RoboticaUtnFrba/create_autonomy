cmake_minimum_required(VERSION 2.8.3)
project(ca_hector)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
		config
		launch
		maps
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
