# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling the lab's robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Development
We recommend developing using `pixi` for dependency management. First, install required system-level dependencies for development by running the following script:
```
bash dev_setup.sh
```
This will do the following:
1. install `docker`
2. install `nvidia-container-toolkit`
3. install `pixi`
4. install `nvm` (required for `pyright`)

### Docker
We recommend developing on `obelisk` using a Docker container (we provide `.devcontainer` support if you use an IDE supporting it). To configure the build for your username for permission-sharing reasons, build the container with the command
```
docker image build --build-arg USERNAME=${USER} --build-arg USER_UID=${UID} -t obelisk .
```
To run the container, run
```
bash startup.sh -n obelisk
```

### Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
