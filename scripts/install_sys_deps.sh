#!/bin/bash

# script flags
autoaccept=false
source_ros=false
no_source_ros=true

for arg in "$@"; do
    case $arg in
        --y)
            autoaccept=true
            shift # Auto-accepts installation
            ;;
        -y)
            autoaccept=true
            shift # Auto-accepts installation
            ;;
        --source-ros)
            source_ros=true
            shift # Sources base ROS in ~/.bashrc
            ;;
        --no-source-ros)
            no_source_ros=true
            shift # Does not source base ROS in ~/.bashrc
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--y] [--source-ros]"
            exit 1
            ;;
    esac
done

if [ "$autoaccept" = true ]; then
    REPLY='y'
else
    read -p $'\033[1;33mThis script will install system dependencies that modify your local filesystem! Continue? [y/n]\033[0m' -n 1 -r
    echo
fi

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "\033[1;33mNot installing local system dependencies!\033[0m"
else
    # basic deps
    sudo apt-get update && sudo apt-get install -y \
        curl \
        git \
        mesa-common-dev \
        python3-dev \
        python3-pip \
        build-essential \
        cmake \
        libglfw3-dev \
        locales

    # ros-related deps
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt-get update -y && \
    sudo apt-get install -y \
        ros-humble-ros-base \
        ros-dev-tools \
        ros-humble-rosidl-generator-cpp \
        ros-humble-rosidl-default-generators \
        ros-humble-rmw-cyclonedds-cpp
    source /opt/ros/humble/setup.bash

    # parse the user's response to adding the ROS source command to ~/.bashrc - specifying no source takes precedence
    if [ "$no_source_ros" = true ]; then
        REPLY='n'
    elif [ "$source_ros" = true ]; then
        REPLY='y'
    else
        read -p $'\033[1;33mROS 2 has been installed. Would you like to add the source command to your .bashrc file? [y/n]\033[0m' -n 1 -r
        echo
    fi

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        echo -e "\033[1;32mROS 2 source command added to .bashrc!\033[0m"
    else
        echo -e "\033[1;33mROS 2 source command not added to .bashrc. You will need to source it manually!\033[0m"
    fi

    # python-specific deps
    pip install -U \
        colcon-common-extensions \
        "ruamel.yaml"
    cd $OBELISK_ROOT/obelisk/python
    pip install -e .
    cd $OBELISK_ROOT

    echo -e "\033[1;32mSystem dependencies installed successfully!\033[0m"
fi
