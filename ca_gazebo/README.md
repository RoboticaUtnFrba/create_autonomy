# Gazebo environments

This directory contains the different environments to run the simulation
within.

## TODO
Each enviroment launchfile has a given offset that compensates for the
create's wheel odometry so that the robot starts at (0,0).

These offsets are hardcoded for amcl, so when using a different localization
package please check if the environment has the offset parameters set for it.
For lama, use `./launch/create_house.launch` as example.
