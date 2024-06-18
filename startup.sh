#!/bin/bash
join=false
while getopts n:j flag
do
    case "${flag}" in
        n) image_name_or_id=${OPTARG};;
        j) join=true;;
    esac
done
xhost +local:docker

# make directory if doesn't exist
mkdir -p /tmp/build
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if [ "$join" = false ] ; then

        # checks whether system has nvidia GPUs
        if (($(nvidia-smi -L | wc -l) > 0)); then
            echo "nvidia GPUs detected! Using nvidia runtime..."
            docker run --rm -it --net=host --ipc=host -e DISPLAY=$DISPLAY \
            --security-opt seccomp=unconfined --cap-add=NET_ADMIN \
            -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v $PWD:/home/placeholder/obelisk \
            -v $HOME/.Xauthority:/root/.Xauthority:rw \
            -e NVIDIA_DRIVER_CAPABILITIES=all \
            --privileged --runtime nvidia --gpus all \
            --name=$image_name_or_id \
            $image_name_or_id /bin/bash

        else
            echo "No nvidia GPUs detected! Using default runtime..."
            docker run --rm -it --net=host --ipc=host -e DISPLAY=$DISPLAY \
            --security-opt seccomp=unconfined --cap-add=NET_ADMIN \
            -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v $PWD:/home/placeholder/obelisk \
            -v $HOME/.Xauthority:/root/.Xauthority:rw \
            --privileged --name=$image_name_or_id \
            $image_name_or_id /bin/bash
        fi
    else
        # join a running container with all the right permissions
        docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        $image_name_or_id /bin/bash
    fi
fi
xhost -local:docker
