# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# build args
ARG USER=user
ARG UID=1000
ARG GID=1000

ARG OBELISK_ROOT=/
ARG OBELISK_DOCKER_BASIC=false
ARG OBELISK_DOCKER_CYCLONE_PERF=false
ARG OBELISK_DOCKER_LEAP=false
ARG OBELISK_DOCKER_ZED=false
ARG OBELISK_DOCKER_PIXI=false

ARG OBELISK_DOCKER_GROUP_LEAP=false
ARG OBELISK_DOCKER_GROUP_ZED=false

# accompanying env variables
ENV USER=$USER
ENV UID=$UID
ENV GID=$GID

ENV OBELISK_ROOT=$OBELISK_ROOT
ENV OBELISK_DOCKER_BASIC=$OBELISK_DOCKER_BASIC
ENV OBELISK_DOCKER_CYCLONE_PERF=$OBELISK_DOCKER_CYCLONE_PERF
ENV OBELISK_DOCKER_LEAP=$OBELISK_DOCKER_LEAP
ENV OBELISK_DOCKER_ZED=$OBELISK_DOCKER_ZED
ENV OBELISK_DOCKER_PIXI=$OBELISK_DOCKER_PIXI

ENV OBELISK_DOCKER_GROUP_LEAP=$OBELISK_DOCKER_GROUP_LEAP
ENV OBELISK_DOCKER_GROUP_ZED=$OBELISK_DOCKER_GROUP_ZED

ENV XDG_RUNTIME_DIR=/run/user/${UID}

# set timezone (used by ROS dependencies)
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# mandatory dependencies that are not handled by pixi
RUN apt-get update && apt-get install -y \
    curl \
    git \
    mesa-common-dev \
    locales \
    iputils-ping \
    evtest \
    sudo && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US.UTF-8

# create non-root user with sudo privileges
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# conditionally install dependencies based on build args
# source base ROS if basic deps are installed and you are not using pixi
COPY install_sys_deps.sh /tmp/install_sys_deps.sh
RUN FLAGS=""; \
    [ "$OBELISK_DOCKER_BASIC" = "true" ] && FLAGS="$FLAGS --basic"; \
    [ "$OBELISK_DOCKER_CYCLONE_PERF" = "true" ] && FLAGS="$FLAGS --cyclone-perf"; \
    [ "$OBELISK_DOCKER_BASIC" = "true" ] && [ "$OBELISK_DOCKER_PIXI" = "false" ] && FLAGS="$FLAGS --source-ros"; \
    [ "$OBELISK_DOCKER_LEAP" = "true" ] && FLAGS="$FLAGS --leap"; \
    [ "$OBELISK_DOCKER_ZED" = "true" ] && FLAGS="$FLAGS --zed"; \
    [ "$OBELISK_DOCKER_UNITREE" = "true" ] && FLAGS="$FLAGS --unitree"; \
    bash /tmp/install_sys_deps.sh $FLAGS && \
    sudo rm /tmp/install_sys_deps.sh

# conditional configure groups based on build args
COPY config_groups.sh /tmp/config_groups.sh
RUN FLAGS=""; \
    [ "$OBELISK_DOCKER_GROUP_LEAP" = "true" ] && FLAGS="$FLAGS --leap"; \
    [ "$OBELISK_DOCKER_GROUP_ZED" = "true" ] && FLAGS="$FLAGS --zed"; \
    bash /tmp/config_groups.sh $FLAGS && \
    sudo rm /tmp/config_groups.sh

# switch to new user and workdir
USER ${UID}

# run user setup script in Dockerfile
# these are deps that can/should be installed without root access
# obelisk aliases are mounted, so we don't create them here
COPY user_setup.sh /tmp/user_setup.sh
RUN FLAGS=""; \
    [ "$OBELISK_DOCKER_PIXI" = "true" ] && FLAGS="$FLAGS --pixi"; \
    source /tmp/user_setup.sh $FLAGS && \
    sudo rm /tmp/user_setup.sh

WORKDIR /home/${USER}
