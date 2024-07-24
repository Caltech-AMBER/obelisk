#!/bin/bash

# script flags
autoaccept=false

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
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--y] [--source-ros]"
            exit 1
            ;;
    esac
done

# check whether the ZED SDK is already installed
temp_file=$(mktemp)
pip list > "$temp_file"
if grep -q pyzed "$temp_file"; then
    echo -e "\033[1;33mZED SDK already installed!\033[0m"
    rm "$temp_file"
    exit 0
fi
rm "$temp_file"

# if OBELISK_ZED=true, then install ZED SDK (set from pixi or setup.sh)
if [ "$OBELISK_ZED" = true ]; then
    autoaccept=true
    echo -e "\033[1;32mInstalling ZED SDK...\033[0m"
fi

# if the user uses the -y flag, install ZED SDK
if [ "$autoaccept" = true ]; then
    REPLY='y'
else
    read -p $'\033[1;33mThis script will install system dependencies that modify your local filesystem! Continue? [y/n]\033[0m' -n 1 -r
    echo
fi

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "\033[1;33mNot installing local system dependencies!\033[0m"
else
    # opencv requires: libegl1, libopengl0
    # zed sdk installation requires: lsb-release, udev, wget, zstd
    sudo apt-get update && sudo apt-get install -y \
        libegl1 \
        libopengl0 \
        lsb-release \
        udev \
        wget \
        zstd

    # installing ZED SDK
    if [ ! -d /etc/udev/rules.d ]; then
        sudo mkdir -p /etc/udev/rules.d
    fi
    sudo /lib/systemd/systemd-udevd --daemon  # starting a udev daemon
    wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O $OBELISK_ROOT/tmp/zed/ubuntu22  # download installer
    sudo chmod +x $OBELISK_ROOT/tmp/zed/ubuntu22  # make installer executable
    $OBELISK_ROOT/tmp/zed/ubuntu22 -- silent skip_cuda skip_od_module skip_hub skip_tools  # run installer
    sudo chown -R $USER:$USER /usr/local/zed  # change ownership of zed sdk
    sudo udevadm control --reload-rules && sudo udevadm trigger  # activating udev rules
    rm -r $OBELISK_ROOT/tmp  # remove installer
fi
