# Getting Started
All instructions in this section assume you are using Ubuntu 22.04. The development dependencies should be installed when `source dev_setup.sh` is run from the repository root.

There are two options for using `obelisk`. One is to use the same dependency management and containerization tools used for development to ensure reproducibility. The other is to simply install `obelisk` as a dependency of your project without using these tools. We will explain how to do both here.

## Option 1: Quick Installation
To install the `obelisk` ROS2 packages, if you have installed `pixi`, you can run
```
pixi run source_obelisk --environment ros2-install
```
This will allow you to use the packages in the `obelisk_ws` directory as an underlay for your own ROS2 packages.

Similarly, if you have ROS2 locally installed already, you can instead run in the `obelisk_ws` directory
```
colcon build --symlink-install
echo "source install/setup.sh" >> ~/.bashrc
```
to build our packages and manually install them as an underlay.

<!-- TODO(@ahl): when the `obelisk_py` and `obelisk_cpp` packages have content, include instructions for how to install them as well. -->

## Option 2: Isolation
<!-- TODO(@ahl): look this over and verify how to use isolation to do dependency management. Update docs in a separate PR. -->
<!-- Once you have cloned this repository, you can run in the repo root:
```
docker compose -f docker/docker-compose.yml run --build obelisk
```
This will load a Docker container with the repository root mounted into the container.

If your downstream package is managed with `pixi`, you can install `obelisk` by [specifying the link to this repository](https://pixi.sh/latest/reference/project_configuration/#version-specification) in your `pixi.toml` file. -->
