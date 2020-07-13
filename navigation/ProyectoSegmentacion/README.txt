Para correr el script: 
- Desde docker: en create_ws ejecutar -> 

export LOCALIZATION=pure_localization 
export RVIZ=true  
export GUI=false 
export RVIZ_CONFIG=navigation 
export LOCAL_PLANNER=teb
export LASER=rplidar

roslaunch ca_gazebo create_house.launch

- En otra terminal de docker : rosrun ProyectoSegmentacion move_base.py 
