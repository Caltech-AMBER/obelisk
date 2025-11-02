# Development of Obelisk

This page concerns details about developing the core of Obelisk, not how to use Obelisk in a downstream project.

-- This page is a work in progress as we continue to update Obelisk --

## Overview
We use the following tools for development:
* Bash shell (some of the scripts will modify your `.bashrc`)
* Docker and Docker Compose for isolating system states and for development containers
* [`ruff`](https://docs.astral.sh/ruff/) and [`pyright`](https://github.com/microsoft/pyright) for linting, code-style enforcement, and type checking of Python code
* [`clang-tidy`](https://clang.llvm.org/extra/clang-tidy/) and [`clang-format`](https://clang.llvm.org/docs/ClangFormat.html) for C++ code
* [`pre-commit`](https://pre-commit.com/) for registering code checks as commit hooks
* [`pytest`](https://docs.pytest.org/en/8.2.x/) for python code tests

We ensure that all code checks pass in a CI workflow before allowing PRs to merge into `main`.

## Getting Started
To work in Obelisk, both as a dev and as a user, we suggest working in a docker container. Previously we had also supported `pixi`, but due to some dependency conflicts/installation issues, we have for now reverted to a docker only build.

First, we need to setup obelisk. The following commands will (1) make sure docker is installed (2) set flags to make sure that the docker build script grabs the correct dependencies (these will NOT be installed on your local machine, only in the docker) (3) tell Obelisk what other options you want and (4) setup obelisk aliases. Please note that the aliases file will be modified on your local machine then mounted into docker.

```
source setup.sh --dev-setup --unitree --mujoco
```

Technically the `--unitree` flag is optional, but it is required to interface with the Unitree SDK. The `--mujoco` flag is needed to run simulations.

Sometimes we have found that .bash_aliases is a folder. For this to work, you will need to delete that and make sure that a single file is created.

If you have just installed docker for the first time, you may need to run
```
sudo usermod -aG docker $USER
newgrp docker
```

Now, there are a few options. We suggest working with obelisk from within a dev container through an IDE. VSCode has good support for this and the Jetbrains IDEs have this feature in beta. If you take this (suggested) route, then you will want to open the the dev container in the IDE using the provided `.devcontainer`.

To build the development Docker container, run from the repo root:
```
docker compose -f docker/docker-compose.yml run --build obelisk
```
To enter the container without rebuilding or to join from a different terminal window, run
```
docker compose -f docker/docker-compose.yml run obelisk
```
If you are running on a machine with no Nvidia GPUs, you can instead run
```
docker compose -f docker/docker-compose-no-gpu.yml run --build obelisk
```

Either way, your repository root will be mounted into the container at the exact same file location, your username, user ID, and group ID will be shared with the container, and you can freely run `sudo` without supplying a password for any command.

<!-- 
Once ready for development, you should start the `pixi` shell, which is like activating a `conda` environment (but `pixi` environments are project/directory-specific, not shell-wide). To do this, run in the repo root:
```
# if your machine has a gpu
pixi shell -e dev

# if your machine has no gpu
pixi shell -e dev-no-gpu
```
The environment `dev` contains the most critical development dependencies. However, you can view the available environment sets in the `pixi.toml` to start a different environment if you would like. For example, if you are only updating the docs, you can do this by setting the environment flag to `docs`.

While in the `pixi` shell and/or Docker container, all changes made in the repository or to the `~/.bashrc` file persist in your local file system. You can also use `git` with no issue to push changes. -->

## Building and Running the ROS Stack
All of the following commands should be run from within the `devcontainer`.
<!-- and within the `dev` pixi virtual environment (which can be entered by running `pixi shell -e dev`). -->

We now need to activate obelisk. Assuming you installed the aliases, you can now run
```
obk
```
The first time this is run it also builds obelisk. This activation command sources all the relevant scripts (like ROS) so that the stack can be used without hassle.

You can also build everything with the terminal command:
```
obk-build
```
this will also be sure to build everything in the correct order.

To launch a ROS stack we can use the following commands.

In a seperate terminal, we can run a ROS stack with:
```
obk-launch config=<config file> device=<device>
```

Specifically, for the dummy examples this looks like:
```
obk-launch config=dummy_cpp.yaml device=onboard
```

All the documentation for the Obelisk terminal aliases can be found [here](obelisk_terminal_aliases.md).

## C++ Code Structure
The C++ libraries in `obelisk/cpp` are built with CMake.

All `obelisk` C++ libraries are placed in `obelisk/`. Each library should have its own folder. Within that folder there should be an `include` folder, a `CMakeLists.txt`, and the source files (i.e., not the header files - those go in `include`).

(testing)=
### Testing
Unit tests are managed with [`Catch2`](https://github.com/catchorg/Catch2). Ultimately we plan to run the tests with [`CTest`](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Testing%20With%20CMake%20and%20CTest.html), and therefore all unit tests need to be registered with `CTest`, see [here](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md). The tests are all placed within `tests/tests_cpp`.
