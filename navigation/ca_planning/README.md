# ca_planning packages

## Jump Point Search

https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html

https://github.com/KumarRobotics/jps3d

https://github.com/daggarwa/AStar_Plugin_ROS/blob/master/src/astar_planner/src/AStarPlanner.cpp#L75

https://github.com/ros-planning/navigation/blob/melodic-devel/navfn/src/navfn_ros.cpp

http://wiki.ros.org/costmap_2d

https://github.com/KumarRobotics/jps3d/compare/master...ICRA2017:reproducible

https://github.com/ros-planning/navigation/issues/147

https://answers.ros.org/question/276935/move_base-make_plan-service-crashes-transport-error/

```bash
GUI=false VERBOSE=true GLOBAL_PLANNER=jps RVIZ_CONFIG=navigation LOCALIZATION=amcl roslaunch ca_gazebo create_house.launch
```

## Backtrace with gdb

- Build `move_base` from source

Install gdb:

```bash
sudo apt-get install -y gdb
```

Compile the workspace with `Debug` symbols:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

Enable backtrace:

```bash
export BACKTRACE=true
```

Run the simulation:

```bash
GUI=false VERBOSE=true RVIZ=true LOCALIZATION=amcl roslaunch ca_gazebo create_house.launch
```

Add a breakpoint (Shift + Insert to paste the following code):

```bash
break /create_ws/src/create_autonomy/navigation/ca_planning/src/jps_ros.cpp:180
```

Cheat sheet: https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf
Costmap: https://github.com/stonier/cost_map/blob/devel/README.md

## Testing

```bash
rosrun ca_planning test_distance_map_planner_2d `rospack find ca_planning`/test/data/corridor.yaml
```
