# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USER=user
ARG UID=1000
ARG GID=1000
ARG OBELISK_ROOT=/
ENV USER=$USER
ENV UID=$UID
ENV GID=$GID
ENV OBELISK_ROOT=$OBELISK_ROOT

# set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# basic dependencies from docker_setup.sh (up until sudo and below)
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    cmake \
    clang-tools-12 \
    nano \
    vim \
    git \
    python3-dev \
    python-is-python3 \
    python3-pip \
    python3-argcomplete \
    mesa-utils \
    x11-apps \
    libyaml-dev \
    mesa-common-dev \
    libglfw3-dev \
    locales \
    sudo && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US.UTF-8

# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# add user to dialout group to enable serial port access from the container
RUN sudo usermod -a -G dialout ${USER}

# DEBUG
##############################################
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
#         -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
#         http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
#         tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
#     apt-get update -y
# RUN apt-get install -y \
#     ros-humble-ros-base \
#     ros-dev-tools \
#     ros-humble-rosidl-generator-cpp \
#     ros-humble-rosidl-default-generators
# RUN apt-get install -y \
#     ros-humble-rmw-cyclonedds-cpp
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
##############################################

# switch to new user and workdir
USER ${UID}

# run docker setup script in Dockerfile
COPY docker_setup.sh /tmp/docker_setup.sh
RUN source /tmp/docker_setup.sh --skip-docker && \
    sudo rm /tmp/docker_setup.sh

# add local user binary folder to PATH variable
ENV PATH="${PATH}:/home/${USER}/.local/bin"
ENV XDG_RUNTIME_DIR=/run/user/${UID}
WORKDIR /home/${USER}
