# Development

## Overview
We use the following tools for development:
* Bash shell (some of the scripts will modify your `.bashrc`)
* Docker and Docker Compose for isolating system states and for development containers
* [`pixi`](https://pixi.sh/latest/) for package-level dependency management across platforms
* [`ruff`](https://docs.astral.sh/ruff/) and [`pyright`](https://github.com/microsoft/pyright) for linting, code-style enforcement, and type checking of Python code
* [`clang-tidy`](https://clang.llvm.org/extra/clang-tidy/) and [`clang-format`](https://clang.llvm.org/docs/ClangFormat.html) for C++ code
* [`pre-commit`](https://pre-commit.com/) for registering code checks as commit hooks
* [`pytest`](https://docs.pytest.org/en/8.2.x/) for python code tests

We ensure that all code checks pass in a CI workflow before allowing PRs to merge into `main`.

## Getting Started
First, there are local dependencies required to use our development pipeline. Run the following script from the repo root:
```
source dev_setup.sh --dev-sys-deps
newgrp docker
```
This will install `docker`, `nvidia-container-toolkit`, `pixi`, `uv`, and `nvm` (required for `pyright`). It also sets the `OBELISK_ROOT` environment variable to the path of the repository root on your local filesystem and installs some useful aliases to the `.bashrc`. If you do not want to install `docker` or the development dependencies, instead just run
```
source dev_setup.sh --skip-docker
```

Optionally, to build the development Docker container, run from the repo root:
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
If your IDE supports it, you can instead use the provided `.devcontainer` to launch this container (VScode provides good support, and at the time of writing, `devcontainers` are a beta feature in JetBrains IDEs). Either way, your repository root will be mounted into the container at the exact same file location, your username, user ID, and group ID will be shared with the container, and you can freely run `sudo` without supplying a password for any command.

Once ready for development, you should start the `pixi` shell, which is like activating a `conda` environment (but `pixi` environments are project/directory-specific, not shell-wide). To do this, run in the repo root:
```
# if your machine has a gpu
pixi shell -e dev

# if your machine has no gpu
pixi shell -e dev-no-gpu
```
The environment `dev` contains the most critical development dependencies. However, you can view the available environment sets in the `pixi.toml` to start a different environment if you would like. For example, if you are only updating the docs, you can do this by setting the environment flag to `docs`.

While in the `pixi` shell and/or Docker container, all changes made in the repository or to the `~/.bashrc` file persist in your local file system. You can also use `git` with no issue to push changes.

## Building and Running the ROS Stack
All of the following commands should be run from within the `devcontainer` and within the `dev` pixi virtual environment (which can be entered by running `pixi shell -e dev`).

Build all the messages:
```
pixi run messages-build
```

Build all the ROS packages (including the messages):
```
pixi run ros-build
```
***Note (as of 9/10/2024): if you have never built the workspace before, you will be required to build the messages first before running `ros-build`. This is because the messages are a dependency for the `ObeliskCpp` library and thus must be built first so they can be used. After you have built the ros workspace once, all future updates only require a `ros-build` command.***

In a seperate terminal, we can run a ROS stack with:
```
obk-launch config_file_path=<config file> device_name=<device>
```

All the documentation for the Obelisk terminal aliases can be found [here](obelisk_terminal_aliases.md).

## Building and Running C++ Code
We can easily run C++ code using `pixi`. From within the `dev` enviroment, the available commands are:
- `cpp-ctest` will run all tests registered with `CTest`. This is the command used in the CI to verify unit tests, so every test should be registered with CTest - see [below](#testing).
- `cpp-test-node` will run all the tests for the main Obelisk library and will do so using the Catch2 framework.
- `cmake` which will re-build the cmake.
- `cpp-build` which will compile the code.

<!-- TODO (@zolkin): move this to a generic pixi section.  -->
Note that all the commands will automatically run the commands they depend on. For this reason, to re-build the cmake, compile the code, and run the tests, all we need to do is run `cpp-ctest`.

We can enter the `dev` enviroment with `pixi shell -e dev` or we can run those commands from the normal shell by pre-pending with `pixi run -e dev`. For example, to run the ctests, `pixi run -e dev cpp-ctest`.

In the future we will add more commands to run other parts of the code.

## C++ Code Structure
The C++ libraries in `obelisk/cpp` are built with CMake.

All `obelisk` C++ libraries are placed in `obelisk/`. Each library should have its own folder. Within that folder there should be an `include` folder, a `CMakeLists.txt`, and the source files (i.e., not the header files - those go in `include`).

### Testing
Unit tests are managed with [`Catch2`](https://github.com/catchorg/Catch2). Ultimately we plan to run the tests with [`CTest`](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Testing%20With%20CMake%20and%20CTest.html), and therefore all unit tests need to be registered with `CTest`, see [here](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md). The tests are all placed within `tests/tests_cpp`.
