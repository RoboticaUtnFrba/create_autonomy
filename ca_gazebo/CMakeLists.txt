cmake_minimum_required(VERSION 3.5.2)
project(ca_gazebo)

add_compile_options(-std=c++11)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# Supress a warning
cmake_policy(SET CMP0054 OLD)
find_package(gazebo 9 REQUIRED)
cmake_policy(SET CMP0054 NEW)

find_package(catkin REQUIRED
  COMPONENTS
    ca_msgs
    gazebo_plugins
    message_generation
    roscpp
    roslint
    std_msgs
    tf
    tf2_ros
)

set(PLUGIN_LIST
  cliff_msg_publisher
  create_battery_plugin
  create_bumper_plugin
  create_cliff_plugin
  create_diff_drive
  create_imu_plugin
  create_virtual_wall_plugin
  model_pose_publisher_plugin
  traffic_light_plugin
  virtual_wall_detector
  world_time_publisher
)

########################
## Message generation ##
########################

# Generate custom messages/services
add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

###################################
###### iRobot Create plugins ######
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PLUGIN_LIST}
  CATKIN_DEPENDS
    ca_msgs
    gazebo_plugins
    message_runtime
    roscpp
    std_msgs
    tf
    tf2_ros
)

foreach(PLUGIN ${PLUGIN_LIST})
  set(TARGET_NAME ${PROJECT_NAME}_${PLUGIN})

  add_library(${TARGET_NAME} src/${PLUGIN}.cpp)

  add_dependencies(${TARGET_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

  target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

  set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME ${PLUGIN})

  install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

#############
## Install ##
#############

install(DIRECTORY
			launch
			test
			worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
  roslint_cpp()

  add_subdirectory(test)
endif()
