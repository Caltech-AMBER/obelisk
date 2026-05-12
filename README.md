# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Setup
Obelisk should be used as a dependency for external robot control code that is written in other downstream projects. There are two options:
1. Use Obelisk in Docker (this is mainly for developing in Obelisk).
2. Use Obelisk on your local filesystem.

### Initial Setup
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
  --dev-setup                  Setup for development of Obelisk (Docker-based).
  --downstream-setup           Setup for downstream use of Obelisk (local install).

  Hardware:
  --zed                        Enable ZED SDK
  --unitree                    Enable Unitree interfaces
  --westwood                   Enable Westwood Robotics THEMIS interface (no system deps; UDP-only)

  Simulation:
  --mujoco                     Enable Mujoco simulation

  --help                       Display this help message and exit
```

Guidance:
* For developing Obelisk inside the dev container: `source setup.sh --dev-setup` (plus any hardware flags you need).
* For consuming Obelisk in a downstream project on your local machine: `source setup.sh --downstream-setup`.
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

### Building Obelisk ROS Packages
Once dependencies are in place, build the Obelisk ROS2 packages with `obk-build` (a colcon wrapper installed by `--dev-setup` / `--downstream-setup`). The recommended workflow is to develop inside the dev container:

```
cd docker
docker compose -f docker-compose.yml run --build obelisk            # GPU
docker compose -f docker-compose-no-gpu.yml run --build obelisk     # no GPU
```

Inside the container, `obk` builds (first time) and sources the workspace; `obk-build` rebuilds after edits; `obk-clean` nukes `build/install/log` if you need a clean slate.

### Using Obelisk
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

## Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
