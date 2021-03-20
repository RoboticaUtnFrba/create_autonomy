# Room Segmentation

```bash
export LOCALIZATION=pure_localization
export RVIZ=true
export GUI=false
export LASER=rplidar

roslaunch ca_gazebo create_house.launch
```

```bash
rosrun ca_room_segmentation move_base.py
```
