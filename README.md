# create_autonomy + Android Assistant

[ROS](http://ros.org) driver for iRobot [Create 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).
This package wraps the C++ library [libcreate][libcreate], which uses iRobot's [Open Interface Specification][oi_spec].

The "android_asssitant" package allows to control the robot through an [Android application](https://github.com/adevitturi/ROS_Voice_Commands_App), by the use of voice commands. The node serves as an interface between the app and move base.
To run the node in the package to communicate with the app, use:

```
roslaunch android_assistant send_goal.launch base_topic:=goal_assistant
```

Where "base_topic" should be the same topic set up in the application's Settings page. In this case, "goal_assistant" is the default topic.

* ROS wiki page: http://wiki.ros.org/create_autonomy
* Support: [ROS Answers (tag: create_autonomy)](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:create_autonomy/page:1/)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Maintainers: [National Technological University - Buenos Aires](https://www.frba.utn.edu.ar/en/)
  * [Emiliano Borghi](https://github.com/eborghi10)

## Build status

![Build Status](https://api.travis-ci.org/RoboticaUtnFrba/create_autonomy.svg?branch=kinetic-devel)

## [Features](docs/FEATURES.md)

## [Installation](docs/INSTALLATION.md)

## [Running the driver](docs/LAUNCH.md)

### [Parameters](docs/PARAMETERS.md)

### [Repository structure](docs/STRUCTURE.md)

## [Commanding your Create](docs/COMMAND.md)

## [Contributions](docs/CONTRIBUTION.md)

[libcreate]:  https://github.com/RoboticaUtnFrba/libcreate
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
