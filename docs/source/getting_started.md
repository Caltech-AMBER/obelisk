# Getting Started
All instructions in this section assume you are using Ubuntu 22.04. The development dependencies should be installed when `dev_setup.sh` is run from the repository root.

## Using Docker
We highly recommend using `docker` to containerize our code. This provides consistent environments and makes it easy to share code with others.

To build a minimal Docker image with only barebones ROS dependencies, run the following command from the `docker` directory:
```
docker image build -t <image_name> -f Dockerfile.minimal .
```
To enter the build container, run the following command in the repository root:
```
bash startup.sh -n <image_name>
```