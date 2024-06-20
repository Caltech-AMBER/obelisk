# Development

## Overview
We use the following tools for development:
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
source dev_setup.sh
newgrp docker
```
This will install `docker`, `nvidia-container-toolkit`, `pixi`, `uv`, and `nvm` (required for `pyright`). It also sets the `OBELISK_ROOT` environment variable to the path of the repository root on your local filesystem.

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
If your IDE supports it, you can instead use the provided `.devcontainer` to launch this container. Either way, your repository root will be mounted into the container at the exact same file location, your username, user ID, and group ID will be shared with the container, and you can freely run `sudo` without supplying a password for any command.

Once ready for development, you should start the `pixi` shell, which is like activating a `conda` environment (but `pixi` environments are project/directory-specific, not shell-wide). To do this, run in the repo root:
```
pixi shell --environment dev
```
The environment `dev` contains the most critical development dependencies. However, you can view the available environment sets in the `pixi.toml` to start a different environment if you would like. For example, if you are only updating the docs, you can do this by setting the environment flag to `docs`.

While in the `pixi` shell and/or Docker container, all changes made in the repository or to the `~/.bashrc` file persist in your local file system. You can also use `git` with no issue to push changes.
