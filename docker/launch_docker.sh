#!/bin/bash

CONTAINER=""

while [ "$1" != "" ]; do
    case "$1" in
        -d | --docker )  CONTAINER="$2";  shift;;
    esac
    shift
done

IMAGE_NAME=${CONTAINER}
SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=`readlink -f ${SCRIPTS_DIR}/../../`

NVIDIA_FLAG=""
if [[ $IMAGE_NAME = *"nvidia"* ]]; then
  NVIDIA_FLAG="--runtime=nvidia"
fi

xhost +
docker run -it \
    --privileged --rm \
    "--ipc=host" \
    "--cap-add=IPC_LOCK" \
    "--cap-add=sys_nice" \
    "--network=host" \
    --env="DISPLAY"  \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${REPO_DIR}:/create_ws/src" \
    --device=/dev/input/js0 \
    --user="$(id -u):$(id -g)" \
    -e ROS_HOSTNAME=localhost \
    -e ROS_MASTER_URI=http://localhost:11311 \
    $NVIDIA_FLAG \
    $IMAGE_NAME
