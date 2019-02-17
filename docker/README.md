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

Install nvidia-docker (to get HW acceleration) https://github.com/NVIDIA/nvidia-docker/wiki

## Step 3: Creating the container

This repository contain the Dockerfile. Move into the directory containing the file and type

The command below will **create** the container from the base image if it doesn't exist and log you in. 

      ```bash
      $ make create-melodic-gazebo9
      ```

## Step 4: Start the container

To make it easier, I created the launcher **launch_docker.sh** (you might need to call **chmod +x ./launch_docker.sh** first).

      ```bash
      $ ./launch_docker.sh
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