FROM ros:melodic-ros-base
LABEL maintainer="Emiliano Borghi"

ENV ROS_DISTRO melodic

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    apt-utils \
    curl \
    dialog \
    software-properties-common \
    tmux \
    wget

# Setup environment
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-gazebo-* \
  && rm -rf /var/lib/apt/lists/*

# ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

# Setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys for ROS
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Setup sources.list for Gazebo
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list

# Setup keys for Gazebo
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python3-vcstool

# Install ROS packages
RUN apt-get update && \
    apt-get install -y \
    gazebo9 \
    libgazebo9-dev

RUN apt-get clean -y && \
    apt-get autoremove -y && \
    apt-get autoclean -y && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Install dependencies for gscam and visual odometry
RUN apt-get update && \
    apt-get install -y \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev

# Install ROS dependencies
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-amcl \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ecl-threads \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-move-base \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-yocs-controllers \
    ros-${ROS_DISTRO}-xacro \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Compile RTIMULib2 and IMU dependencies
RUN mkdir -p /home/${USER}/catkin_ws/src
WORKDIR /home/${USER}/catkin_ws/src
RUN git clone https://github.com/RoboticaUtnFrba/RTIMULib2.git && \
    git clone https://github.com/RoboticaUtnFrba/i2c_imu.git
WORKDIR /home/${USER}/catkin_ws/src/RTIMULib2/Linux
RUN apt-get update && apt-get install -y libqt4-dev
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    ldconfig

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc

WORKDIR /home/${USER}/catkin_ws

CMD [ "tmux" ]