# Getting Started
Obelisk is a suite of tools designed to make robotics development simpler. The centerpiece of Obelisk is ROS, which allows Obelisk to be extended and used in real-world robotics tasks.

There are two main options for using Obelisk: (1) use Obelisk from within a docker container, or (2) run Obelisk locally on your system. We strongly recommond option (1) as Obelisk requires a number of complex dependencies. If you choose to take option (2) then there could be installation issues and/or system clutter. Using option (1) also makes your code less suscepitble to issues caused by adjusting your local machine's configuraiton.

## Setting up Obelisk with Docker
With this option we suggest that the docker container acts merely as your simulation/hardware interface. The ideal setup has your algorithm code as a seperate library, not dependent on ROS, which can then be added to this docker container. This also helps seperate code relating to running a robot with the algorithms.

A sample docker file is as follows:
```
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
    libeigen3-dev \
    x11-apps \
    locales \
    iputils-ping \
    evtest \
    sudo && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US.UTF-8

# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# switch to new user and workdir
USER ${UID}


RUN echo "Pulling Obelisk..."
# clone Obelisk into this docker image at the $OBELISK_ROOT path
RUN git clone -b v0.1.0-alpha https://github.com/Caltech-AMBER/obelisk.git /home/${USER}/obelisk
ENV OBELISK_ROOT=/home/${USER}/obelisk

# Install other dependencies here...

# you must run the setup script from a directory where the user has permissions
# run docker setup script in Dockerfile
WORKDIR /home/${USER}
RUN source /home/${USER}/obelisk/setup.sh --install-sys-deps --cyclone-perf --obk-aliases --basic

WORKDIR /home/${USER}
```

This docker file installs a number of basic tools and pulls obelisk and installs it. Note that here we can choose a specific Obelisk version to install if desired.

The line
```
RUN source /home/${USER}/obelisk/setup.sh --install-sys-deps --cyclone-perf --obk-aliases --basic
```
is critical as this runs the Obelisk setup script within the docker container. It is possible that you may need Obelisk to be configured differently, and you can do that by adjusting those flags in the Dockerfile.

The flag options are:
```
Options:
  --recommended                Apply recommended system-level changes
                               (cyclone performance optimizations, pixi, obelisk aliases)

  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --leap                       Enables LEAP hand dependencies
  --zed                        Enables ZED SDK
  --unitree                    Enables the unitree interfaces

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --config-groups              Configures user groups associated with hardware

  --install-sys-deps           Installs system dependencies
  --source-ros                 Sources base ROS in ~/.bashrc (only used if --install-sys-deps)

  --pixi                       Install pixi
  --obk-aliases                Add obelisk aliases to the ~/.bash_aliases file

  --help                       Display this help message and exit
  ```

Since we are already in the docker container at this point, we want to skip over the `docker-install` and `install-sys-deps-docker` options. We want to treat the docker container as the local machine so that the setup script installs everything within the docker container. `--install-sys-deps --cyclone-perf --obk-aliases --basic` with any hardware interfaces like `--unitree` or `--leap` are the suggested installation flags.

Now we can enter into the docker container (or even better, this can be setup as a dev container). Within this docker container we can handle all development that interfaces our algorithm with Obelisk.

## Basic Obelisk Usage
From here on out we assume that Obelisk is installed and you are in the associated docker container and the Obelisk aliases have been installed.

Now we need to activate Obelisk:
```
obk
```
This command sources all the relevant files (like ROS) so that the entire stack is ready to use. The first time this command runs Obelisk is also built. You can see more about the aliases [here](obelisk_terminal_aliases.md).

To build obelisk any other time we use
```
obk-build
```
If you are using a stable version of Obelisk, you should only need to run this once. If there are any build issues in subsequent builds we suggest running `obk-clean` followed by `obk-build` to clear the build cache and rebuild.

Once Obelisk is activated and built we can run a basic example to make sure everything is working as expected.
```
obk-launch config_file_path=dummy_cpp.yaml device_name=onboard
```
At this point you should see a Mujoco simulation running.

`obk-launch` is an important alias that calls the obelisk launch file. Assuming your code conforms to Obelisk standards (discussing in a later section), an `obk-launch` command should be all you need to bring the full stack, including simulation/hardware.