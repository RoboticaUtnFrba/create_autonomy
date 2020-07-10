FROM create_ubuntu_bionic
LABEL maintainer="Emiliano Borghi"

ENV ROS1_DISTRO="melodic"
ENV USER="create"

USER root

# Setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys for ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS1_DISTRO}-desktop \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Reduce roslaunch timeout
RUN sudo sed -i 's/_TIMEOUT_SIGTERM = 2.0/_TIMEOUT_SIGTERM = 0.5/g' /opt/ros/${ROS1_DISTRO}/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
RUN sudo sed -i 's/_TIMEOUT_SIGINT  = 15.0/_TIMEOUT_SIGINT  = 1.0/g' /opt/ros/${ROS1_DISTRO}/lib/python2.7/dist-packages/roslaunch/nodeprocess.py

# Initialize rosdep
RUN rosdep init

RUN pip install --upgrade pip

USER $USER
RUN echo ". /opt/ros/${ROS1_DISTRO}/setup.bash" >> /home/${USER}/.bashrc
RUN echo ". /create_ws/src/create_autonomy/ca_bringup/scripts/robot_network_config.sh" >> /home/${USER}/.bashrc
USER root

# Workspace
RUN mkdir -p /create_ws/src/ && \
    chown -R $USER /create_ws
USER $USER
WORKDIR /create_ws/
RUN rosdep update
