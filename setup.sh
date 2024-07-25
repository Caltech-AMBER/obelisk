#!/bin/bash

# script flags
basic=false
cyclone_perf=false
leap=false
zed=false

docker_install=false
install_sys_deps_docker=false

install_sys_deps=false
source_ros=false

pixi=false
obk_aliases=false

for arg in "$@"; do
    case $arg in
        --recommended)
            cyclone_perf=true
            pixi=true
            obk_aliases=true
            shift  # Allows recommended system-level changes
            ;;

        # dependency groups to configure
        --basic)
            basic=true
            shift  # Enables basic dependencies necessary for Obelisk locally
            ;;
        --cyclone-perf)
            cyclone_perf=true
            shift # Enables cyclone performance optimizations
            ;;
        --leap)
            leap=true
            shift  # Enables LEAP hand dependencies
            ;;
        --zed)
            zed=true
            shift  # Enables ZED SDK
            ;;

        # docker setup
        --docker-install)
            docker_install=true
            shift  # Installs Docker and nvidia-container-toolkit
            ;;
        --install-sys-deps-docker)
            install_sys_deps_docker=true
            shift  # Installs system dependencies in Docker
            ;;

        # system-level deps
        --install-sys-deps)
            install_sys_deps=true
            shift  # Installs system dependencies
            ;;
        --source-ros)
            source_ros=true
            shift # Sources base ROS in ~/.bashrc (only used if --install-sys-deps)
            ;;

        # user setup
        --pixi)
            pixi=true
            shift # Installs pixi
            ;;
        --obk-aliases)
            obk_aliases=true
            shift # Adds obelisk aliases to the ~/.bash_aliases file
            ;;

        # help
        --help)
            echo "Usage: source setup.sh [OPTIONS]

Options:
  --recommended                Apply recommended system-level changes
                               (cyclone performance optimizations, pixi, obelisk aliases)

  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --leap                       Enables LEAP hand dependencies
  --zed                        Enables ZED SDK

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --install-sys-deps           Installs system dependencies
  --source-ros                 Sources base ROS in ~/.bashrc (only used if --install-sys-deps)

  --pixi                       Install pixi
  --obk-aliases                Add obelisk aliases to the ~/.bash_aliases file

  --help                       Display this help message and exit
"
            shift
            return
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg. Run 'source setup.sh --help' for more information."
            return
            ;;
    esac
done

# set OBELISK_ROOT to be the directory containing this script
export OBELISK_ROOT=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# docker setup
# if you want to use the ZED camera, you always need the local deps
if [ "$install_sys_deps_docker" = true ]; then
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$basic" = true ] && echo "--docker-basic") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$leap" = true ] && echo "--docker-leap") \
        $([ "$zed" = true ] && echo "--docker-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi")
else
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$zed" = true ] && echo "--docker-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi")
fi

# system-level deps
if [ "$install_sys_deps" = true ]; then
    source $OBELISK_ROOT/scripts/install_sys_deps.sh --zed \
        $([ "$basic" = true ] && echo "--basic") \
        $([ "$cyclone_perf" = true ] && echo "--cyclone-perf") \
        $([ "$source_ros" = true ] && echo "--source-ros") \
        $([ "$leap" = true ] && echo "--leap")
fi

# run user-specific setup
source $OBELISK_ROOT/scripts/user_setup.sh \
    $([ "$pixi" = true ] && echo "--pixi") \
    $([ "$leap" = true ] && echo "--leap") \
    $([ "$zed" = true ] && echo "--zed") \
    $([ "$obk_aliases" = true ] && echo "--obk-aliases")
