# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USER=user
ARG UID=1000
ARG GID=1000
ARG OBELISK_ROOT=/
ARG OBELISK_BASIC=false
ARG OBELISK_ZED=false

ENV USER=$USER
ENV UID=$UID
ENV GID=$GID
ENV OBELISK_ROOT=$OBELISK_ROOT
ENV OBELISK_BASIC=$OBELISK_BASIC
ENV OBELISK_ZED=$OBELISK_ZED

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

# dialout group allows serial port access
# video group allows ZED camera access (also need to chown /usr/local/zed; done in script)
RUN usermod -a -G dialout ${USER} && \
    usermod -a -G video ${USER}

# conditionally install dependencies based on build args
COPY install_sys_deps.sh /tmp/install_sys_deps.sh
RUN FLAGS=""; \
    [ "$OBELISK_BASIC" = "true" ] && FLAGS="$FLAGS --basic"; \
    [ "$OBELISK_ZED" = "true" ] && FLAGS="$FLAGS --zed"; \
    bash /tmp/install_sys_deps.sh $FLAGS && \
    sudo rm /tmp/install_sys_deps.sh

# switch to new user and workdir
USER ${UID}

# run docker setup script in Dockerfile
# these are deps that can/should be installed without root access
COPY docker_setup.sh /tmp/docker_setup.sh
RUN source /tmp/docker_setup.sh --pixi --cyclone-perf && \
    sudo rm /tmp/docker_setup.sh

WORKDIR /home/${USER}
