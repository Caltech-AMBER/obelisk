# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Setup
Obelisk should be used as a dependency for external robot control code that is written in other downstream projects. There are a few options:
1. Use Obelisk in Docker (this is mainly for developing in Obelisk).
2. Use Obelisk on your local filesystem.
3. Use Obelisk in a project that uses `pixi`.

### Initial Setup
Initial setup proceeds by running the `setup.sh` script in the repository root. This script has the ability to make changes to your local dependencies - all such changes are opt-in. The available options are:
```
source setup.sh [--no-skip-docker] [--pixi] [--cyclone-perf] [--bash-aliases] [--obk-aliases]
```
* The `--no-skip-docker` flag installs `docker` and `nvidia-container-toolkit` on your local filesystem. You should only specify this if you want to develop in a containerized setting.
* The `--pixi` flag installs `pixi` on your local filesystem.
* The `--cyclone-perf` flag adds [performance optimizations for Cyclone DDS](https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations) in the `/etc/sysctl.d/60-cyclonedds.conf` file on your local filesystem. You should  specify this if you plan to use Obelisk in a non-containerized environment.
* The `--bash-aliases` flag will check if `~/.bash_aliases` is sourced in the `~/.bashrc` file (and will add it if not already in there), and will create the `~/.bash_aliases` file if it doesn't already exist. This is a very benign flag, so we recommend using it.
* The `--obk-aliases` flag will add useful Obelisk aliases to the `~/.bash_aliases` file. **We highly recommend using this flag.**
* If you trust us, you can use the `--all` flag to just opt-in to all of these dependencies.
If you're more cautious, we recommend running
```
source setup.sh --pixi --bash-aliases --obk-aliases
```
If you're installing `docker` for the first time using this script, you also need to run afterwards
```
newgrp docker
```

### Building Obelisk
Next, since Obelisk acts as a dependency for a downstream ROS2 project, you have to build it. You can either build it on your local filesystem or in a virtual environment that we manage using `pixi`.

* If you are building it on your local filesystem, you need some minimal set of local dependencies. You can install these by running
    ```
    bash scripts/install_sys_deps.sh
    ```
    This will prompt you to confirm changes to your local filesystem. To auto-accept the installations, use the `-y` flag. To source the base ROS installation in the `~/.bashrc` without prompting, use the `--source-ros` flag. To not source it without prompting, use the `--no-source-ros` flag.

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

## Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
