# Getting Started -- Using Obelisk in a Downstream Project
Obelisk is a suite of tools designed to make robotics development simpler. The centerpiece of Obelisk is ROS, which allows Obelisk to be extended and used in real-world robotics tasks.

There are two main options for using Obelisk: (1) use Obelisk from within a docker container, or (2) run Obelisk locally on your system. We strongly recommond option (1) as Obelisk requires a number of complex dependencies. If you choose to take option (2) then there could be installation issues and/or system clutter. Using option (1) also makes your code less suscepitble to issues caused by adjusting your local machine's configuraiton.

For developing the core Obelisk library, please see Development.

## Setting up Obelisk with Docker
The key idea with using Docker is to make installation/setup easier and repeatable on everyone's computer. There are a number of dependencies, especially ROS related, that have been known to cause issues when installed without any careful planning. By using a docker container we can mitigate these issues.
That being said, ideally this Docker is just for Obelisk deployment. The ideal setup has your algorithm code as a seperate library, not dependent on ROS, which can then be added to this docker container. This also helps seperate code relating to running a robot with the underlying algorithms.

The recommended docker folder structure for using Obelisk is:
```
docker/
├── .env
├── Dockerfile
├── docker-compose.yml
```

And optionally, if you are using a devcontainer:
```
.devcontainer/
├── gpu/
  ├── devcontainer.json
├── no_gpu/
  ├── devcontainer.json
```
with the `.devcontainer` folder at the time layer as the `docker` folder.

### Sample `Dockerfile`
A sample `Dockerfile` is as follows:
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
RUN git clone -b main https://github.com/Caltech-AMBER/obelisk.git /home/${USER}/obelisk
ENV OBELISK_ROOT=/home/${USER}/obelisk

# Install other dependencies here...

# you must run the setup script from a directory where the user has permissions
# run docker setup script in Dockerfile
WORKDIR /home/${USER}
RUN source /home/${USER}/obelisk/setup.sh --downstream-setup --mujoco

WORKDIR /home/${USER}
```

This docker file installs a number of basic tools and pulls obelisk and installs it. Note that here we can choose a specific Obelisk version to install if desired using `-b` flag in the clone - for now we are just installing main.

The line
```
RUN source /home/${USER}/obelisk/setup.sh --downstream-setup
```
is critical as this runs the Obelisk setup script within the docker container. It is possible that you may need Obelisk to be configured differently, and you can do that by adjusting those flags in the `Dockerfile`.

The flag options are:
```
Options:
  --dev-setup                  Setup for development of Obelisk.

  --downstream-setup           Setup for downstream use of Obelisk - suggested to build this in a docker container.

  Hardware options:
  --zed                        Enables ZED SDK
  --unitree                    Enables the unitree interfaces

  Simulation options:
  --mujoco                     Enables Mujoco simulation. For now, without this flag there will be no simulation of any type.

  Other options:
  --help                       Display this help message and exit
```

We only want to use `--dev-setup` if we are preparing to develop core obelisk features, and in that case we want to use the docker container that comes in the Obelisk repo. We can choose hardware interfaces to be installed with `--unitree` or `--zed`. When deploying on a robot, you may not want a simulation environment, so you can not pass `--mujoco`.

### Sample `docker-compose.yml`
Below is a sample `docker-compose.yml` that assumes you want to use an Nvidia GPU with Obelisk.
```
services:
  downstream_obk:
    shm_size: '12gb'
    build:
      context: .
      args:
        USER: $USER
        UID: $UID
        GID: $GID
      dockerfile: Dockerfile
    network_mode: host
    ipc: host
    environment:
      NVIDIA_DRIVER_CAPABILITIES: all
      DISPLAY: $DISPLAY
      USER: $USER
      UID: $UID
      GID: $GID
      QT_X11_NO_MITSHM: 1
    security_opt:
      - seccomp=unconfined
    cap_add:
      - NET_ADMIN
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $HOME/.Xauthority:$HOME/.Xauthority:rw
      - $HOME/.bashrc:$HOME/.bashrc
      - $HOME/.ssh:$HOME/.ssh
    ports:
      - 7007:7007
      - 10000:10000
    privileged: true
    runtime: nvidia
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [ gpu ]
    working_dir: $ROBOT_RL_ROOT
    stdin_open: true
    tty: true
    command: /bin/bash
```

### Sample `.env`
```
USER=${USER:-$(id -un)}
UID=1000
GID=1000
```

You may need to adjust the `UID` of `GID` depending on the users in your computer, but in general this works for most people.

### Sample `devcontainer.json`
This file can be used to create a `devcontainer` in Vscode, which is currently the suggested way of developing in docker containers.
```
{
    "name": "Obelisk GPU Dev Container",
    "dockerComposeFile": "../../docker/docker-compose.yml",
    "service": "downstream_obk",
    "workspaceFolder": "${localWorkspaceFolder}",
    "shutdownAction": "stopCompose",
    "customizations": {
        "vscode": {
            "extensions": [
                "mutantdino.resourcemonitor",
                "ms-azuretools.vscode-docker",
                "nvidia.nsight-vscode-edition",
                "ritwickdey.liveserver",
                "ms-vscode.cmake-tools",
                "ms-python.python",
                "charliermarsh.ruff"
            ]
        }
    },
}
```

### Creating the Docker Container
Now we can enter into the docker container (or even better, this can be setup as a dev container). Within this docker container we can handle all development that interfaces our algorithm with Obelisk.

Run 
```
docker compose -f docker/docker-compose.yml run --build downstream_obk
```
to build the container from the terminal.

Or if you are using a devcontainer you can rebuild and enter the container from within the VScode extension.

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