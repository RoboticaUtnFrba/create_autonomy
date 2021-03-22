cmake_minimum_required(VERSION 2.8.3)
project(ca_room_segmentation)

add_compile_options(-std=c++11)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
set(QT_LIBRARIES Qt5::Widgets)
set(QTVERSION ${Qt5Widgets_VERSION})

find_package(catkin REQUIRED COMPONENTS
	python_qt_binding
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/execute
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
