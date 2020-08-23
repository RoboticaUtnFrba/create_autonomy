FROM create_ros_melodic
LABEL maintainer="Emiliano Borghi"

ENV USER="create"

USER root

# Setup sources.list for Gazebo
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Setup keys for Gazebo
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# Install Gazebo 9
RUN apt-get update && \
    apt-get install -y \
    gazebo9 \
    libgazebo9-dev

# Install custom cartographer
ENV CARTO_WS="/home/${USER}/catkin_ws"
RUN mkdir ${CARTO_WS}
WORKDIR ${CARTO_WS}
RUN apt-get install -y \
        ninja-build \
        python-rosdep \
        python-wstool \
        stow
RUN wstool init src && \
    wstool merge -t src \
        https://raw.githubusercontent.com/RoboticaUtnFrba/cartographer_ros/master/cartographer_ros.rosinstall && \
    wstool update -t src
RUN src/cartographer/scripts/install_proto3.sh --local
RUN src/cartographer/scripts/install_abseil.sh
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=${ROS1_DISTRO} -yi -r --os=ubuntu:bionic
# Change again to root in order to install in /opt
USER root
RUN /bin/bash -c "source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    catkin_make_isolated --install --use-ninja --install-space /opt/ros/${ROS1_DISTRO}"
RUN rm -rf ${CARTO_WS}

# Install RTIMULib
WORKDIR ${WS_DIR}
ENV IMU_SCRIPT="install_rtimulib.sh"
COPY sensors/ca_imu/scripts/${IMU_SCRIPT} ${IMU_SCRIPT}
RUN sudo apt-get install -qq -y libqt4-dev
# Disable OpenGL for this application
ENV CI=true
RUN /bin/bash ${IMU_SCRIPT}
RUN rm -r ${IMU_SCRIPT}

# Install dependencies with vcs
# http://wiki.ros.org/catkin/Tutorials/using_a_workspace#Installing_Packages
ENV DEPS_WS=/deps_ws
RUN mkdir -p ${DEPS_WS}/src
WORKDIR ${DEPS_WS}/src
RUN apt-get install -y python3-vcstool
COPY dependencies.repos ${DEPS_WS}/src/dependencies.repos
RUN vcs import . < dependencies.repos
WORKDIR ${DEPS_WS}
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=${ROS1_DISTRO} -yi -r --os=ubuntu:bionic
USER root
RUN /bin/bash -c ". /opt/ros/${ROS1_DISTRO}/setup.bash; \
        catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS1_DISTRO}; \
        cd build; make install"
RUN rm -r ${DEPS_WS}

# Install dependencies from apt
# https://unix.stackexchange.com/a/391112
ENV DEPS_FILE="dependencies.txt"
COPY ${DEPS_FILE} ${DEPS_FILE}
RUN apt-get update && \
    xargs -a ${DEPS_FILE} apt-get install -y
RUN rm ${DEPS_FILE}

USER $USER
