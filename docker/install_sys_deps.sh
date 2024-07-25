#!/bin/bash

# script flags
basic=false
python=false
source_ros=false
zed=false

for arg in "$@"; do
    case $arg in
        --basic)
            basic=true
            shift
            ;;  # Installs basic system dependencies
        --python)
            python=true
            shift
            ;;  # Installs python dependencies
        --source-ros)
            source_ros=true
            shift # Sources base ROS in ~/.bashrc
            ;;
        --zed)
            zed=true
            shift # Installs ZED SDK
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]

Options:
  --basic              Install basic dependencies
  --python             Install python dependencies
  --source-ros         Source base ROS in ~/.bashrc
  --zed                Install ZED SDK

  --help               Display this help message and exit
"
            shift
            exit 0
            ;;

        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--y] [--source-ros]"
            exit 1
            ;;
    esac
done

# [1] basic dependencies
if [ "$basic" = true ]; then
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
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rviz-visual-tools \
        ros-humble-foxglove-bridge \
        ros-humble-dynamixel-sdk
    source /opt/ros/humble/setup.bash

    echo -e "\033[1;32mSystem dependencies installed successfully!\033[0m"
else
    echo -e "\033[1;33mNot installing basic system dependencies!\033[0m"
fi

# [2] python dependencies
if [ "$python" = true ]; then
    # check whether the OBELISK_ROOT directory exists
    if [ -d $OBELISK_ROOT ]; then
        pip install -U \
            colcon-common-extensions \
            "ruamel.yaml" \
            mujoco
        pip install -e $OBELISK_ROOT/obelisk/python
        cd $OBELISK_ROOT
    else
        echo -e "\033[1;33mOBELISK_ROOT directory does not exist!\033[0m"
    fi

    echo -e "\033[1;32mPython dependencies installed successfully!\033[0m"
else
    echo -e "\033[1;33mNot installing python dependencies!\033[0m"
fi

# [3] sourcing base ROS2
if [ "$source_ros" = true ]; then
    # check whether /opt/ros/humble/setup.bash exists; if so, source
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        echo -e "\033[1;32mROS 2 sourced successfully!\033[0m"
    else
        echo -e "\033[1;33mROS 2 not sourced because /opt/ros/humble/setup.bash does not exist!\033[0m"
    fi
fi

# [4] ZED SDK installation
if [ "$zed" = true ]; then
    # check whether the ZED SDK is already installed
    skip_zed=false
    temp_file=$(mktemp)
    pip list > "$temp_file"
    if grep -q pyzed "$temp_file"; then
        echo -e "\033[1;33mZED SDK already installed!\033[0m"
        rm "$temp_file"
        skip_zed=true
    fi
    rm "$temp_file"

    if [ "$skip_zed" = false ]; then
        # opencv requires: libegl1, libopengl0
        # zed sdk installation requires: lsb-release, udev, wget, zstd
        sudo apt-get update && sudo apt-get install -y \
            libegl1 \
            libopengl0 \
            lsb-release \
            udev \
            wget \
            zstd

        # [July 24, 2024] installing ZED SDK (version 4.1.3)
        if [ ! -d /etc/udev/rules.d ]; then
            sudo mkdir -p /etc/udev/rules.d
        fi
        sudo /lib/systemd/systemd-udevd --daemon  # starting a udev daemon
        wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O ubuntu22  # download installer
        sudo chmod +x ubuntu22  # make installer executable
        ./ubuntu22 -- silent skip_cuda skip_od_module skip_hub skip_tools  # run installer
        sudo chown -R $USER:$USER /usr/local/zed  # change ownership of zed sdk
        sudo udevadm control --reload-rules && sudo udevadm trigger  # activating udev rules
        rm ubuntu22  # remove installer
    else
        echo -e "\033[1;33mZED SDK already installed!\033[0m"
    fi
else
    echo -e "\033[1;33mNot installing ZED SDK!\033[0m"
fi
