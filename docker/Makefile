# If you want to rebuild all images, set this to --no-cache from the commandline
DOCKER_ARGS?=
USER_ID=$(shell id -u)

# ROS Melodic

.PHONY: create_ubuntu_bionic
create_ubuntu_bionic:
	docker build ${DOCKER_ARGS} --build-arg uid=$(USER_ID) -t create_ubuntu_bionic create_ubuntu_bionic

.PHONY: create_ros_melodic
create_ros_melodic: create_ubuntu_bionic
	docker build ${DOCKER_ARGS} -t create_ros_melodic create_ros_melodic

.PHONY: create_ros_melodic_gazebo9
create_ros_melodic_gazebo9: create_ros_melodic
	docker build ${DOCKER_ARGS} -t create_ros_melodic_gazebo9 create_ros_melodic_gazebo9

.PHONY: create_melodic_nvidia
create_melodic_nvidia: create_ros_melodic_gazebo9
	docker build ${DOCKER_ARGS} -t create_melodic_nvidia create_melodic_nvidia
