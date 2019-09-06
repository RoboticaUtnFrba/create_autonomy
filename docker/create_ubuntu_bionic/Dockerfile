FROM ubuntu:18.04
ARG uid
LABEL maintainer="Emiliano Borghi"

# Setup environment
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    apt-utils \
    bash-completion \
    build-essential \
    curl \
    git \
    mesa-utils \
    pkg-config \
    python-vcstools \
    python-pip \
    software-properties-common \
    ssh \
    sudo \
    tmux \
    wget

RUN apt-get update && \
    apt-get install -y dirmngr && \
    apt-get upgrade -y && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Create a user with passwordless sudo
RUN adduser --gecos "Development User" --disabled-password -u ${uid} create
RUN adduser create sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER root

CMD tmux
