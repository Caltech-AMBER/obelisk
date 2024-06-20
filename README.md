# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling the lab's robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Development
We recommend developing using `pixi` for dependency management. First, install required system-level dependencies for development and setting environment variables by running the following script:
```
source dev_setup.sh
newgrp docker
```
This will do the following:
1. install `docker`
2. install `nvidia-container-toolkit`
3. install `pixi`
4. install `uv`
5. install `nvm` (required for `pyright`)
6. set the `OBELISK_ROOT` environment variable to the repo root

### Docker
We recommend developing on `obelisk` using a Docker container (we provide `.devcontainer` support if you use an IDE supporting it). You can also bring up a container using docker-compose as follows from the repository root:
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

### Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
