# Room Segmentation

```bash
export LOCALIZATION=amcl
export RVIZ=true
export GUI=false
export LASER=rplidar

roslaunch ca_gazebo create_house.launch
```

Execute the room segmentation action server:

```bash
roslaunch ipa_room_segmentation room_segmentation_action_server.launch
```

```bash
rosrun ca_room_segmentation execute
```

More information about the package `ipa_room_segmentation` can be found [here](https://github.com/ipa320/ipa_coverage_planning/blob/indigo_dev/ipa_room_segmentation/readme.md) and (here)[http://wiki.ros.org/ipa_room_segmentation].

The document explaining this package is [here](https://drive.google.com/file/d/1zWyM3eSH5mJpbYpZpAOByEis81tZmZ2D/view?usp=sharing) but it's in Spanish.
