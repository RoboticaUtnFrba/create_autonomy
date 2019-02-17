#!/bin/bash 
xhost +

DOCKER_CAPABILITIES="--ipc=host \
                     --cap-add=IPC_LOCK \
                     --cap-add=sys_nice"

DOCKER_NETWORK="--network=host"

docker run -it \
--privileged --rm \
${DOCKER_CAPABILITIES} \
${DOCKER_NETWORK} \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--workdir="/home/$USER/catkin_ws" \
--volume="/home/$USER/catkin_ws/src:/home/$USER/catkin_ws/src" \
--volume="/etc/group:/etc/group:ro" \
--volume="/etc/passwd:/etc/passwd:ro" \
--volume="/etc/shadow:/etc/shadow:ro" \
--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
-e LOCAL_USER_ID=`id -u $USER` \
-e LOCAL_GROUP_ID=`id -g $USER` \
-e LOCAL_GROUP_NAME=`id -gn $USER` \
 create-melodic-gazebo9
