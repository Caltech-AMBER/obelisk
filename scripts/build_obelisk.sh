#!/bin/bash

leap=false
zed=false
verbose=false

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
        --verbose)
            verbose=true
            shift
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--leap] [--zed] [--verbose]"
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
    SKIP_PKGS+=" obelisk_zed_cpp"
fi
VERBOSE_STR=""
if [ "$verbose" = true ]; then
    VERBOSE_STR="--event-handlers console_direct+"
fi

# configuring cmake args
USE_CMAKE_ARGS=false
CMAKE_ARGS=""
if [ "$zed" = true ]; then
    # [NOTE] these arguments will prevent the following error during colcon build:
    #    /usr/bin/ld: warning: libcuda.so.1, needed by /usr/local/zed/lib/libsl_zed.so, not found (try using -rpath or -rpath-link)
    #    /usr/bin/ld: warning: libnvcuvid.so.1, needed by /usr/local/zed/lib/libsl_zed.so, not found (try using -rpath or -rpath-link)
    #    /usr/bin/ld: warning: libnvidia-encode.so.1, needed by /usr/local/zed/lib/libsl_zed.so, not found (try using -rpath or -rpath-link)
    CMAKE_ARGS+=" -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs"
    CMAKE_ARGS+=" -DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\""
    CMAKE_ARGS+=" --no-warn-unused-cli"
fi

# building Obelisk packages
OBELISK_ROOT=$(dirname $(dirname $(readlink -f ${BASH_SOURCE[0]})))

echo -e "\033[1;32mBuilding Obelisk ROS messages...\033[0m"
curr_dir=$(pwd)
cd $OBELISK_ROOT/obelisk_ws

colcon build --symlink-install --parallel-workers $(nproc) \
    $VERBOSE_STR --packages-select $MESSAGE_PKGS

echo -e "\033[1;32mBuilding remainder of Obelisk ROS packages...\033[0m"
source $OBELISK_ROOT/obelisk_ws/install/setup.bash
colcon build --symlink-install --parallel-workers $(nproc) \
    --packages-skip $MESSAGE_PKGS $SKIP_PKGS \
    $VERBOSE_STR ${CMAKE_ARGS:+--cmake-args $CMAKE_ARGS}

source $OBELISK_ROOT/obelisk_ws/install/setup.bash
cd $curr_dir

echo -e "\033[1;32mObelisk built successfully!\033[0m"
