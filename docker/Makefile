# If you want to rebuild all images, set this to --no-cache from the commandline
DOCKER_ARGS?=

.PHONY: create_ubuntu_bionic
create_ubuntu_bionic:
	docker build ${DOCKER_ARGS} -t create_ubuntu_bionic create_ubuntu_bionic

.PHONY: create_ros_melodic
create_ros_melodic: create_ubuntu_bionic
	docker build ${DOCKER_ARGS} -t create_ros_melodic create_ros_melodic

.PHONY: create_gazebo9
create_gazebo9: create_ros_melodic
	docker build ${DOCKER_ARGS} -t create_gazebo9 create_gazebo9

.PHONY: create_nvidia
create_nvidia: create_gazebo9
	docker build ${DOCKER_ARGS} -t create_nvidia create_nvidia
