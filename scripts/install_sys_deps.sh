#!/bin/bash

# --- script flags --- #
# generic deps/installations
basic=false
cyclone_perf=false
source_ros=false

# hardware-specific deps
leap=false
zed=false
unitree=false

for arg in "$@"; do
    case $arg in
        --basic)
            basic=true
            shift
            ;;  # Installs basic system dependencies
        --cyclone-perf)
            cyclone_perf=true
            shift
            ;;  # Enables cyclone performance optimizations
        --source-ros)
            source_ros=true
            shift # Sources base ROS in ~/.bashrc
            ;;
        --zed)
            zed=true
            shift # Installs ZED SDK
            ;;
        --unitree)
            unitree=true
            shift # Sets up the Unitree interfaces
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]

Options:
  --basic              Install basic dependencies
  --cyclone-perf       Enable cyclone performance optimizations
  --source-ros         Source base ROS in ~/.bashrc
  --zed                Install ZED SDK
  --unitree            Sets up the Unitree interface

  --help               Display this help message and exit
"
            shift
            exit 0
            ;;

        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--basic] [--cyclone-perf] [--source-ros] [--leap] [--zed]"
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
        python-is-python3 \
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
        ros-humble-joy              # Need to make sure this is version 3.3.0
    source /opt/ros/humble/setup.bash

    # python deps
    pip install -U \
        colcon-common-extensions \
        "ruamel.yaml" \
        mujoco
    if [ -d $OBELISK_ROOT ]; then
        pip install -e $OBELISK_ROOT/obelisk/python
        echo -e "\033[1;32mOBELISK_ROOT exists, obelisk_py installed as editable!\033[0m"
    else
        pip install git+https://github.com/Caltech-AMBER/obelisk.git#subdirectory=obelisk/python
        echo -e "\033[1;33mOBELISK_ROOT directory does not exist! Installing obelisk_py from GitHub...\033[0m"
    fi

    echo -e "\033[1;32mSystem dependencies installed successfully!\033[0m"
else
    echo -e "\033[1;33mNot installing basic system dependencies!\033[0m"
fi

# [2] enables cyclone performance optimizations
# see: https://github.com/ros2/rmw_cyclonedds?tab=readme-ov-file#performance-recommendations
if [ "$cyclone_perf" = true ]; then
    # check whether /etc/sysctl.d/60-cyclonedds.conf is a directory - if it is, delete it
    if [ -d /etc/sysctl.d/60-cyclonedds.conf ]; then
        sudo rm -rf /etc/sysctl.d/60-cyclonedds.conf
    fi

    # apply performance optimizations
    if ! grep -q "net.core.rmem_max=8388608" /etc/sysctl.d/60-cyclonedds.conf; then
        echo 'net.core.rmem_max=8388608' | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
    fi

    if ! grep -q "net.core.rmem_default=8388608" /etc/sysctl.d/60-cyclonedds.conf; then
        echo 'net.core.rmem_default=8388608' | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
    fi

    echo -e "\033[1;32mCyclone DDS performance optimizations enabled permanently!\033[0m"
else
    echo -e "\033[1;33mCyclone DDS performance optimizations disabled. To enable, pass the --cyclone-perf flag.\033[0m"
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
        # zed sdk installation requires: lsb-release, pciutils, udev, wget, zstd
        sudo apt-get update && sudo apt-get install -y \
            libegl1 \
            libopengl0 \
            lsb-release \
            pciutils \
            udev \
            wget \
            zstd

        # [July 24, 2024] installing ZED SDK, including python version (version 4.1.3)
        if [ ! -d /etc/udev/rules.d ]; then
            sudo mkdir -p /etc/udev/rules.d
        fi
        sudo /lib/systemd/systemd-udevd --daemon  # starting a udev daemon
        wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O ubuntu22  # download installer
        sudo chmod +x ubuntu22  # make installer executable
        ./ubuntu22 -- silent skip_od_module skip_hub skip_tools  # run installer
        sudo chown -R $USER:$USER /usr/local/zed  # change ownership of zed sdk
        sudo udevadm control --reload-rules && sudo udevadm trigger  # activating udev rules
        rm ubuntu22  # remove installer

        # granting permissions for python dist-packages because zed SDK installs a bunch of these necessary for colcon
        sudo chmod -R 755 /usr/local/lib/python3.10/dist-packages/

        echo -e "\033[1;32mZED SDK installed successfully! Run 'newgrp video' if cameras not found.\033[0m"
    else
        echo -e "\033[1;33mZED SDK already installed!\033[0m"
    fi
else
    echo -e "\033[1;33mNot installing ZED SDK!\033[0m"
fi
