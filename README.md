# create_autonomy

[ROS](http://ros.org) driver for iRobot [Create 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).
This package wraps the C++ library [libcreate][libcreate], which uses iRobot's [Open Interface Specification][oi_spec].

* ROS wiki page: http://wiki.ros.org/create_autonomy
* Support: [ROS Answers (tag: create_autonomy)](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:create_autonomy/page:1/)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Maintainers: [National Technological University - Buenos Aires](https://www.frba.utn.edu.ar/en/)
  * [Emiliano Borghi](https://github.com/eborghi10)

## Build status

![Build Status](https://api.travis-ci.org/RoboticaUtnFrba/create_autonomy.svg?branch=kinetic-devel)
[![GitHub issues](https://img.shields.io/github/issues-raw/RoboticaUtnFrba/create_autonomy)](https://github.com/RoboticaUtnFrba/create_autonomy/issues)
[![GitHub](https://img.shields.io/github/license/RoboticaUtnFrba/create_autonomy)](https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/LICENSE)

## [Features](docs/FEATURES.md)

## [Installation](docs/INSTALLATION.md)

## [Running the driver](docs/LAUNCH.md)

### [Parameters](docs/PARAMETERS.md)

### [Repository structure](docs/STRUCTURE.md)

## [Commanding your Create](docs/COMMAND.md)

## [Contributions](docs/CONTRIBUTION.md)

[libcreate]:  https://github.com/RoboticaUtnFrba/libcreate
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf

##############################

source src/create_autonomy/ca_bringup/scripts/robot_network_config.sh
LASER=rplidar RVIZ_CONFIG=localization roslaunch ca_bringup complete.launch

LOCALIZATION=amcl LASER=rplidar RVIZ_CONFIG=navigation GUI=false roslaunch ca_gazebo create_house.launch

http://wiki.ros.org/rviz_imu_plugin
https://roverrobotics.com/blogs/guides/fusing-imu-encoders-with-ros-robot-localization
https://answers.ros.org/question/271566/robot_localization-enhance-local-positioning-with-imu/
http://docs.ros.org/melodic/api/robot_localization/html/configuring_robot_localization.html
https://kapernikov.com/the-ros-robot_localization-package/
https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
