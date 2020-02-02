cmake_minimum_required(VERSION 2.8.3)
project(ca_gazebo)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    ca_msgs
    gazebo_plugins
    gazebo_ros
    roscpp
    roslint
    tf2
    tf2_ros
)

set(trajectory_actor_plugin_name TrajectoryActorPlugin)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES
		create_bumper_plugin
		model_pose_publisher_plugin
		${trajectory_actor_plugin_name}
  CATKIN_DEPENDS
    ca_msgs
)

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

#############################
## Trajectory Actor plugin ##
#############################

# Create the libTrajectoryActorPlugin.so library.
add_library(${trajectory_actor_plugin_name} SHARED
  src/servicesim/TrajectoryActorPlugin.cc
)
target_link_libraries(${trajectory_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
install(TARGETS ${trajectory_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Follow Actor plugin ##
#############################

# Create the libFollowActorPlugin.so library.
set(follow_actor_plugin_name FollowActorPlugin)
add_library(${follow_actor_plugin_name} SHARED
  src/servicesim/FollowActorPlugin.cc
)
add_dependencies(${follow_actor_plugin_name}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${follow_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)
install(TARGETS ${follow_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Collision Actor plugin ##
#############################

# Create the libCollisionActorPlugin.so library.
set(collision_actor_plugin_name CollisionActorPlugin)
add_library(${collision_actor_plugin_name} SHARED
  src/servicesim/CollisionActorPlugin.cc
)
target_link_libraries(${collision_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
install(TARGETS ${collision_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
## Attach Model plugin ##
#############################

# Create the libAttachModelPlugin.so library.
set(attach_model_plugin_name AttachModelPlugin)
add_library(${attach_model_plugin_name} SHARED
  src/servicesim/AttachModelPlugin.cc
)
target_link_libraries(${attach_model_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
install(TARGETS ${attach_model_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############################
###### Vicinity plugin ######
#############################

# Create the libVicinityPlugin.so library.
set(vicinity_plugin_name VicinityPlugin)
add_library(${vicinity_plugin_name} SHARED
  src/servicesim/VicinityPlugin.cc
)
add_dependencies(${vicinity_plugin_name}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${vicinity_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
install(TARGETS ${vicinity_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

###################################
###### iRobot Create plugins ######
###################################

add_library(create_bumper_plugin src/create_bumper_plugin.cpp)
add_dependencies(create_bumper_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_bumper_plugin
	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(model_pose_publisher_plugin src/model_pose_publisher_plugin.cpp)
target_link_libraries(model_pose_publisher_plugin
	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY
			launch
			media
			models
			worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
		create_bumper_plugin
		model_pose_publisher_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()
