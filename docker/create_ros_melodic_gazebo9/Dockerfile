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

# Install dependencies
# https://unix.stackexchange.com/a/391112
COPY packages.txt packages.txt
RUN apt-get update && \
    xargs -a packages.txt apt-get install -y

USER $USER
