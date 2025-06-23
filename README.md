# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

# Quickstart

This section contains Quickstart information for members of the AMBER Lab.

## FR3
Initial setup in the repo root. For AMBER lab members, get the FR3 credentials from Albert or Gary.
```bash
source setup.sh \
    --cyclone-perf \
    --obk-aliases \
    --pixi \
    --fr3 \
    --config-groups \
    --fr3-username=<YOUR_FR3_USERNAME> \
    --fr3-password=<YOUR_FR3_PASSWORD>
```

Enter Docker container + setup in the container:
```bash
cd docker
docker compose -f docker-compose.yml run --build obelisk
```

Inside the Docker container:
```
# build and enter the pixi shell
pixi shell -e dev

# activate obelisk, clean, and rebuild
obk
obk-clean
obk-build
```

# Setup
Obelisk should be used as a dependency for external robot control code that is written in other downstream projects. There are a few options:
1. Use Obelisk in Docker (this is mainly for developing in Obelisk).
2. Use Obelisk on your local filesystem.
3. Use Obelisk in a project that uses `pixi`.

## Initial Setup
Initial setup proceeds by running the `setup.sh` script in the repository root. This script has the ability to make changes to your local dependencies - all such changes are opt-in. **It is very important that you run `setup.sh` using the `source` command, and not `bash`, because there are environment variables that will be sourced!**

This script has the ability to do 4 things:
1. configure a conditional Docker build so that the image has the right dependencies in it to run Obelisk
2. add the user to various groups necessary for running hardware
3. install system dependencies **on the machine running this script**
4. set up user-specific settings, including very useful bash aliases

The options are as follows:
```
source setup.sh [OPTIONS]

Options:
  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --leap                       Enables LEAP hand dependencies
  --zed                        Enables ZED SDK
  --unitree                    Enables Unitree dependencies
  --fr3                        Enables FR3 dependencies

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --config-groups              Configures user groups associated with hardware

  --install-sys-deps           Installs system dependencies
  --source-ros                 Sources base ROS in ~/.bashrc (only used if --install-sys-deps)

  --pixi                       Install pixi
  --obk-aliases                Add obelisk aliases to the ~/.bash_aliases file

  --help                       Display this help message and exit
```

Some guidance/recommendations on choosing flags:
* If you don't have Docker on your machine, use the `--docker-install` flag
* If you are not using Docker, then you should use `--install-sys-deps`
* If you are using Docker, but not pixi, you should also use `--install-sys-deps`
* If you are using Docker, use `--install-sys-deps-docker` if and only if you are **not** using pixi within the container.
* If you are using pixi, regardless of whether you are using Docker, you should **not** use the `--basic` flag (note: you may have to manually install `mesa-common-dev`)
* If you are not using pixi or conda, you should probably use `--source-ros` along with `--install-sys-deps`
* We believe using `--pixi` will make your life easier, but you don't have to use it
* We strongly recommend using `--obk-aliases` and `--cyclone-perf`
* If you are using the LEAP Hand, use `--leap`
* If you are using ZED cameras, use `--zed`. Additionally, you will need to adjust the `udev` permissions on your host machine if you want to use the ZED cameras in a Docker container with a non-root user (if you are acting as root in your container, you probably don't need to do this next step):
* You should run `--config-groups` only if your local user isn't set up to interface with hardware of interest. If you are going to develop in a Docker container, the group settings will be set for you **without** setting this flag. If you are cloning this repo in a Docker container and running the setup script, setup will **only be complete if you do set this flag**.
    ```
    # grab the ZED SDK installer (version 4.1.3, this README written July 25, 2024)
    wget -q https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O zed_installer.run

    # just pull the udev rules out of the installer
    bash ./zed_installer.run --tar -x './99-slabs.rules'  > /dev/null 2>&1

    # copy the rules to the right directory, make them executable, and reload udev permissions
    sudo mv "./99-slabs.rules" "/etc/udev/rules.d/"
    sudo chmod 777 "/etc/udev/rules.d/99-slabs.rules"
    sudo udevadm control --reload-rules && sudo udevadm trigger

    # remove the installer
    rm zed_installer.run
    ```

If you're installing `docker` for the first time using this script, you also need to run afterwards
```
newgrp docker
```

## Building Obelisk ROS Packages
Next, since Obelisk acts as a dependency for a downstream ROS2 project, you have to build it. You can either build it on your local filesystem or in a virtual environment that we manage using `pixi`.

* If you are building it on your local filesystem, you need some minimal set of local dependencies. These should have been installed in the previous step.

    If you have run the initial setup script with the `--obk-aliases` flag, then running
    ```
    obk-build
    ```
    will build the Obelisk ROS2 libraries on your local filesystem.
* If you are building it using `pixi`, we recommend using the Docker containers we provide:
    ```
    # enter the docker container
    cd docker
    docker compose -f [docker-compose.yml | docker-compose-no-gpu.yml] run --build obelisk

    # build and enter the pixi shell
    pixi shell -e [dev | dev-no-gpu]

    # build in the pixi shell
    pixi run ros-clean
    obk-build
    ```

## Using Obelisk
Once Obelisk has been built, you can use it. If you have set up Obelisk using the `--obk-aliases` flag, we provide a very useful command:
```
obk
```
You should run `obk` in any terminal where you need to run Obelisk. It will set up important environment variables to configure the Obelisk stack while also sourcing the built Obelisk packages in that shell. `obk` also runs `obk-build` if it is detected that there is no `install` directory in the Obelisk workspace, so you can use it to first-time build Obelisk. You only need to run `obk-build` if you are developing and changing Obelisk's source code.

To clean Obelisk directories, run
```
obk-clean
```
This will delete cached build files associated with Obelisk. If you have tried building the Obelisk source code multiple times or from different environments/local filesystems, it may be corrupted, and cleaning the installation can help fix issues.

# Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
