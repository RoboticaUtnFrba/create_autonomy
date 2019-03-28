FROM create_ubuntu_bionic
LABEL maintainer="Emiliano Borghi"

ENV ROS1_DISTRO=melodic

USER root

# Setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys for ROS
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    ros-$ROS1_DISTRO-xacro \
    ros-$ROS1_DISTRO-desktop && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Initialize rosdep
RUN rosdep init

# Install camera dependencies
RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev

RUN apt-get update && \
    apt-get install -y \
    ros-$ROS1_DISTRO-amcl \
    ros-$ROS1_DISTRO-camera-calibration \
    ros-$ROS1_DISTRO-controller-manager \
    ros-$ROS1_DISTRO-ecl-threads \
    ros-$ROS1_DISTRO-hardware-interface \
    ros-$ROS1_DISTRO-joint-limits-interface \
    ros-$ROS1_DISTRO-move-base \
    ros-$ROS1_DISTRO-pcl-ros \
    ros-$ROS1_DISTRO-robot-localization \
    ros-$ROS1_DISTRO-transmission-interface \
    ros-$ROS1_DISTRO-yocs-controllers \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

RUN pip install --upgrade pip

USER create
RUN echo ". /opt/ros/$ROS1_DISTRO/setup.bash" >> /home/create/.bashrc
USER root

# Workspace
RUN mkdir -p /create_ws/src/ && \
    chown -R create /create_ws
USER create
WORKDIR /create_ws/
RUN rosdep update
USER root

CMD tmux
