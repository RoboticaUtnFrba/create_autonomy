cmake_minimum_required(VERSION 2.8.3)
project(ca_dashboard)

find_package(catkin REQUIRED COMPONENTS 
  ca_node
  ca_msgs
  diagnostic_msgs
  rospy
  rqt_gui
  rqt_gui_py
  rqt_robot_dashboard
)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/ca_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
