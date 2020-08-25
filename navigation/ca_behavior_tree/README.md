# ca_behavior_tree

Official documentation: https://www.behaviortree.dev/

```bash
# Debug tree
rosrun groot Groot
# Start simulation
GUI=false RVIZ=true LOCALIZATION=amcl roslaunch ca_gazebo create_house.launch
# Execute BT
roslaunch ca_behavior_tree bt.launch
```
