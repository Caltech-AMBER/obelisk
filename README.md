# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Setup
Obelisk should be used as a dependency for external robot control code that is written in other downstream projects. There are a few options:
1. Use Obelisk in Docker (this is mainly for developing in Obelisk).
2. Use Obelisk on your local filesystem.
3. Use Obelisk in a project that uses `pixi`.

### Initial Setup
Initial setup proceeds by running the `setup.sh` script in the repository root. This script has the ability to make changes to your local dependencies - all such changes are opt-in. **It is very important that you run `setup.sh` using the `source` command, and not `bash`, because there are environment variables that will be sourced!**

This script has the ability to do 3 things:
1. configure a conditional Docker build so that the image has the right dependencies in it to run Obelisk
2. install system dependencies **on the machine running this script**
3. set up user-specific settings, including very useful bash aliases

The options are as follows:
```
source setup.sh [OPTIONS]

Options:
  --recommended                Apply recommended system-level changes
                               (cyclone performance optimizations, pixi, obelisk aliases)

  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --leap                       Enables LEAP hand dependencies
  --zed                        Enables ZED SDK

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

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
* If you are using ZED cameras, use `--zed`

If you're installing `docker` for the first time using this script, you also need to run afterwards
```
newgrp docker
```

### Building Obelisk
Next, since Obelisk acts as a dependency for a downstream ROS2 project, you have to build it. You can either build it on your local filesystem or in a virtual environment that we manage using `pixi`.

* If you are building it on your local filesystem, you need some minimal set of local dependencies. You can install these by running
    ```
    bash scripts/install_sys_deps.sh [--basic] [--cyclone-perf] [--source-ros] [--leap] [--zed]
    ```
    where
    * the `--basic` flag installs basic system-level dependencies (recommended). **Warning: if this is run in the Docker container during build time, the OBELISK_ROOT directory will not exist yet, and it will install `obelisk_py` by cloning it from github (instead of installing it as an editable from the mounted repo).**
    * The `--cyclone-perf` flag adds [performance optimizations for Cyclone DDS](https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations) in the `/etc/sysctl.d/60-cyclonedds.conf` file on your local filesystem (recommended)
    * the `--source-ros` flag will add code that auto-sources base ROS2 to your `~/.bashrc`
    * the `--leap` flag will install LEAP hand-related dependencies
    * the `--zed` flag will install the ZED SDK, which can only be installed locally. It also installs the Python SDK - if you are using a virtual environment, **activate it before running this script!**
    These settings are summarized by running `bash scripts/install_sys_deps.sh --help`.

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
