#!/bin/bash

leap=false
zed=false

for arg in "$@"; do
    case $arg in
        # hardware options
        --leap)
            leap=true
            shift  # Adds leap ROS packages to colcon build command
            ;;
        --zed)
            zed=true
            shift  # Adds ZED ROS packages to colcon build command
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--leap] [--zed]"
            exit 1
            ;;
    esac
done

MESSAGE_PKGS=" obelisk_control_msgs obelisk_estimator_msgs obelisk_sensor_msgs obelisk_std_msgs"

# configuring which packages to skip
SKIP_PKGS=""
if [ "$leap" = false ]; then
    SKIP_PKGS+=" obelisk_leap_cpp obelisk_leap_py"
fi
if [ "$zed" = false ]; then
    SKIP_PKGS+=""  # TODO(ahl): fill this once ZED packages are added
fi

# building Obelisk packages
OBELISK_ROOT=$(dirname $(dirname $(readlink -f ${BASH_SOURCE[0]})))

echo -e "\033[1;32mBuilding Obelisk ROS messages...\033[0m"
curr_dir=$(pwd)
cd $OBELISK_ROOT/obelisk_ws
colcon build --symlink-install --parallel-workers $(nproc) \
    --packages-select $MESSAGE_PKGS \
    --packages-skip $SKIP_PKGS

echo -e "\033[1;32mBuilding remainder of Obelisk ROS packages...\033[0m"
source $OBELISK_ROOT/obelisk_ws/install/setup.bash
colcon build --symlink-install --parallel-workers $(nproc) \
    --packages-skip $MESSAGE_PKGS $SKIP_PKGS
source $OBELISK_ROOT/obelisk_ws/install/setup.bash
cd $curr_dir

echo -e "\033[1;32mObelisk built successfully!\033[0m"
