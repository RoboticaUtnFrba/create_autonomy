# ROS Melodic + Gazebo 9 on Docker

## Step 1: Install Docker
[Install docker](https://docs.docker.com/engine/installation/linux/ubuntu/) and [configure after postintalling it](https://docs.docker.com/install/linux/linux-postinstall/).

To run docker without super user:

        ```bash
        $ sudo groupadd docker
        $ sudo gpasswd -a ${USER} docker
        $ sudo service docker restart
        ```

## Step 2: Use NVIDIA acceleration

### Install [nvidia-docker2](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))

#### Prerequisites

1. GNU/Linux x86_64 with kernel version > 3.10
2. Docker >= 1.12
3. NVIDIA GPU with Architecture > Fermi (2.1)
4. NVIDIA drivers ~= 361.93 (untested on older versions)

#### Removing nvidia-docker 1.0

Version 1.0 of the nvidia-docker package must be cleanly removed before continuing.
You must stop and remove all containers started with nvidia-docker 1.0.

        ```bash
        $ docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
        $ sudo apt-get purge nvidia-docker
        ```

#### Installing version 2.0

Make sure you have installed the [NVIDIA driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver).

If you have a custom `/etc/docker/daemon.json`, the `nvidia-docker2` package might override it.

Install the repository for your distribution by following the instructions here.

      ```bash
      $ wcurl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
        sudo apt-key add -
      $ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
      $ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-docker.list
      $ sudo apt-get update
      ```

Add Docker's official GPG key.

      ```bash
      $ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
        sudo apt-key add -
      ```

Install the `nvidia-docker2` package and reload the Docker daemon configuration:

      ```bash
      $ sudo apt-get install nvidia-docker2
      $ sudo pkill -SIGHUP dockerd
      ```

## Step 3: Creating the container

This repository contains the Dockerfile. Move into the directory containing the file and type

The command below will **create** the container from the base image if it doesn't exist and log you in.

      ```bash
      $ ./build
      ```

The build script will automatically detect whether you are running with a nvidia setup or not. However, it's also possible to force the build script to build for a nvidia setup.

      ```bash
      $ ./build --force-nvidia
      ```

## Step 4: Start the container

To run the container, you can use the run script:

      ```bash
      $ ./run
      ```

Every time you launch the Docker container, you'll need to compile the workspace and source:

      ```bash
      $ catkin_make -DCMAKE_BUILD_TYPE=Release -j4
      $ source devel/setup.bash
      ```

# References

* http://wiki.ros.org/docker/Tutorials/Docker
* http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
* http://wiki.ros.org/docker/Tutorials/GUI
