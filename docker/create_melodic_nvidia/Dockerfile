FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
FROM create_ros_melodic_gazebo9
LABEL maintainer="Emiliano Borghi"

ENV USER="create"

# Installing OpenGL for nvidia-docker2
# https://stackoverflow.com/a/53823600

COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# Remove sudo message when openning a new terminal
# https://askubuntu.com/a/22614
RUN touch ~/.sudo_as_admin_successful

USER $USER
