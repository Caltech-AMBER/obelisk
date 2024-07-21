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
ENV XDG_RUNTIME_DIR=/run/user/${UID}

# set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# basic dependencies that are not handled by pixi
RUN apt-get update && apt-get install -y \
    curl \
    git \
    mesa-common-dev \
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
RUN usermod -a -G dialout ${USER}

# switch to new user and workdir
USER ${UID}

# run docker setup script in Dockerfile
COPY docker_setup.sh /tmp/docker_setup.sh
RUN source /tmp/docker_setup.sh --cyclone-perf && \
    sudo rm /tmp/docker_setup.sh

WORKDIR /home/${USER}
