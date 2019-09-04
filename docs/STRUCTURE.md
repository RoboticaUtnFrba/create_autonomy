# Repository structure

1. `ca_bringup`

    Package containing only a set of launch files whose goal is to start the robot.
    Follows the [REP 144](http://www.ros.org/reps/rep-0144.html#special-cases).

2. `ca_bumper2pc`

    Publish bumpers and cliff sensors events as points in a pointcloud, so the navigation stack can use them.

3. `ca_description`

    Package containing the URDF and meshes of a robot.

4. `ca_driver`

    ROS driver for iRobot Create 2, based on [libcreate](https://github.com/RoboticaUtnFrba/libcreate).

5. `ca_gazebo`

    Gazebo [worlds](http://sdformat.org/spec?ver=1.6&elem=world), [models](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot) and [plugins](http://gazebosim.org/tutorials/?tut=plugins_hello_world).

6. `ca_msgs`

    Custom messages.

7. `ca_node`

    Nodelet to execute `ca_driver` script.

8. `ca_tools`

    Launch and configuration files for common accessories.

9. `create_autonomy`

    Project [metapackage](http://wiki.ros.org/Metapackages).

10. `docker`

    Docker configuration.

11. `docs`

    Documentation files.

12. `navigation`

    a. `ca_localization`

    Implements localization using Kalman filters through the `robot_localization` package.

    b. `ca_move_base`

    Implements navigation capabilities through `move_base`.

    c. `ca_safety_controller`

    A controller ensuring the safe operation of iRobot Create 2.

    The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two, Create is commanded to move back. In the latter case, Create is stopped.

    d. `ca_slam`

    SLAM capabilities with different algorithms.

    e. `ca_visual_odometry`

    Visual odometry package.

13. `sensors`

    a. `ca_imu`

    IMU driver for [MPU9255](https://store.invensense.com/products/detail/MPU9255-InvenSense-Inc/520231/).
