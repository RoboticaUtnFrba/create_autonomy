FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
FROM create_ros_melodic_gazebo9
LABEL maintainer="Emiliano Borghi"

LABEL ROS1_DISTRO="melodic"

# Installing OpenGL for nvidia-docker2
# https://stackoverflow.com/a/53823600

COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

USER root
RUN apt-get update

# Added dependencies
RUN apt-get install -y \
  libopencv-dev \
  ros-$ROS1_DISTRO-dwa-local-planner \
  # ros-$ROS1_DISTRO-eband-local-planner \
  ros-$ROS1_DISTRO-global-planner \
  ros-$ROS1_DISTRO-image-proc \
  ros-$ROS1_DISTRO-joy \
  ros-$ROS1_DISTRO-joy-teleop \
  # ros-$ROS1_DISTRO-libdlib \
  ros-$ROS1_DISTRO-map-server \
  ros-$ROS1_DISTRO-mbf-costmap-core \
  ros-$ROS1_DISTRO-mbf-msgs \
  ros-$ROS1_DISTRO-move-base-flex \
  # ros-$ROS1_DISTRO-opengm \
  ros-$ROS1_DISTRO-sbpl-lattice-planner \
  # ros-$ROS1_DISTRO-slam-gmapping \
  ros-$ROS1_DISTRO-smach-viewer

USER create
