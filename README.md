# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Setup
Obelisk should be used as a dependency for external robot control code that is written in other downstream projects. There are a few options:
1. Use Obelisk in Docker (this is mainly for developing in Obelisk).
2. Use Obelisk on your local filesystem.
3. Use Obelisk in a project that uses `pixi`.

### Initial Setup
Initial setup proceeds by running the `setup.sh` script in the repository root. This script has the ability to make changes to your local dependencies - all such changes are opt-in. **It is very important that you run `setup.sh` using the `source` command, and not `bash`, because there are environment variables that will be sourced!** The available options are:
```
source setup.sh [--docker] [--docker-basic] [--docker-cyclone-perf] [--docker-zed] [--pixi] [--obk-aliases]
```
* The `--docker` flag installs `docker` and `nvidia-container-toolkit` on your local filesystem. You should only specify this if you want to develop in a containerized setting.
* The `--docker-<dep>` flags set environment variables `OBELISK_<DEP>=true` in the file `docker/.env`. This exposes certain system-level dependencies as build arguments for your Docker image. For example, `source setup.sh --docker-zed` will set `OBELISK_ZED=true`, so the ZED SDK will be built into the image. The `--docker-basic` flag will install basic dependencies required for the function of Obelisk, and is recommended if you aren't developing in `pixi`.
* The `--pixi` flag installs `pixi` and associates it with the current user.
* The `--obk-aliases` flag will add useful Obelisk aliases to the `~/.bash_aliases` file. If `~/.bash_aliases` is not already sourced in your `~/.bashrc`, it will also add that. **We highly recommend using this flag.**
* If you trust us, you can use the `--all` flag to just opt-in to all of these dependencies.
If you're more cautious, we recommend running
```
source setup.sh --recommended
```
This is equivalent to using the `--pixi`, `--obk-aliases`, and `--docker-cyclone-perf` flags.

If you're installing `docker` for the first time using this script, you also need to run afterwards
```
newgrp docker
```

### Building Obelisk
Next, since Obelisk acts as a dependency for a downstream ROS2 project, you have to build it. You can either build it on your local filesystem or in a virtual environment that we manage using `pixi`.

* If you are building it on your local filesystem, you need some minimal set of local dependencies. You can install these by running
    ```
    bash scripts/install_sys_deps.sh [--basic] [--cyclone-perf] [--source-ros] [--zed]
    ```
    where
    * the `--basic` flag installs basic system-level dependencies (recommended). **Warning: if this is run in the Docker container during build time, the OBELISK_ROOT directory will not exist yet, and it will install `obelisk_py` by cloning it from github (instead of installing it as an editable from the mounted repo).**
    * The `--cyclone-perf` flag adds [performance optimizations for Cyclone DDS](https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations) in the `/etc/sysctl.d/60-cyclonedds.conf` file on your local filesystem (recommended)
    * the `--source-ros` flag will add code that auto-sources base ROS2 to your `~/.bashrc`
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
