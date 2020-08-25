#catkin_lint: ignore duplicate_cmd
#catkin_lint: ignore uninstalled_export_lib
cmake_minimum_required(VERSION 3.5.2)
project(ca_gazebo)

add_compile_options(-std=c++11)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(gazebo 9 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    brass_gazebo_battery
    ca_msgs
    gazebo_plugins
    gazebo_ros
    roscpp
    roslint
    tf
    tf2
    tf2_ros
)

set(SERVICESIM_PLUGIN_LIST
  AttachModelPlugin
  CollisionActorPlugin
  FollowActorPlugin
  TrajectoryActorPlugin
  VicinityPlugin
)

set(PLUGIN_LIST
  cliff_msg_publisher
  create_bumper_plugin
  create_cliff_plugin
  create_diff_drive
  create_virtual_wall_plugin
  model_pose_publisher_plugin
  traffic_light_plugin
  virtual_wall_detector
  world_time_publisher
)

###########
## Build ##
###########

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

########################
## ServiceSim plugins ##
########################

foreach(PLUGIN ${SERVICESIM_PLUGIN_LIST})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PLUGIN}
    CATKIN_DEPENDS ca_msgs
  )

  add_library(${PLUGIN} SHARED src/servicesim/${PLUGIN}.cc)

  add_dependencies(${PLUGIN} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(${PLUGIN}
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

  install(TARGETS ${PLUGIN}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endforeach()

###################################
###### iRobot Create plugins ######
###################################

foreach(PLUGIN ${PLUGIN_LIST})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PLUGIN}
    CATKIN_DEPENDS ca_msgs
  )

  add_library(${PLUGIN} src/${PLUGIN}.cpp)

  add_dependencies(${PLUGIN} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(${PLUGIN}
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

  install(TARGETS ${TARGET}
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
			media
			models
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

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Tests list
  foreach(TEST
    hz
    publish
    static
  )
    add_rostest(test/${TEST}.test)
  endforeach()
endif()
