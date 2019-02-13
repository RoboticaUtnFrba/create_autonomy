FROM ros:melodic-ros-base
LABEL maintainer="Emiliano Borghi"

# Setup environment
# RUN apt-get update && apt-get install -y locales
# RUN locale-gen en_US.UTF-8
# ENV \
#   LANG=en_US.UTF-8 \
#   DEBIAN_FRONTEND=noninteractive \
#   TERM=xterm

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    apt-utils \
    curl \
    software-properties-common \
    tmux \
    wget 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-melodic-gazebo-* \
  && rm -rf /var/lib/apt/lists/*

# ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

# Setup sources.list for ROS
# RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys for ROS
# RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Setup sources.list for Gazebo
# RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list

# Setup keys for Gazebo
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# Install bootstrap tools
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     python-rosdep \
#     python-rosinstall

# Install ROS packages
# RUN apt-get update && \
#     apt-get install --no-install-recommends -y \
#     gazebo9* \
#     ros-melodic-gazebo-ros-pkgs \
#     apt-get clean && \
#     apt-get autoremove && \
#     apt-get autoclean && \
#     rm -rf /var/lib/apt/lists/* && \
#     rm -rf /usr/share/doc

# Install ROS dependencies
RUN apt-get update && \
    apt-get upgrade -y \
    apt-get install -y \
    ros-melodic-hardware-interface \
    ros-melodic-controller-manager \
    ros-melodic-joint-limits-interface \
    ros-melodic-transmission-interface \
    ros-melodic-rosbridge-server \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

WORKDIR /home/${USER}/catkin_ws

CMD [ "source devel/setup.bash && tmux" ]