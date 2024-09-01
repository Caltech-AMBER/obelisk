#!/bin/bash

# script flags
basic=false
cyclone_perf=false
leap=false
zed=false
zed_ai=false

docker_install=false
install_sys_deps_docker=false

config_groups=false

install_sys_deps=false
source_ros=false

pixi=false
obk_aliases=false

# Variable for mj-source-dir
mj_source_dir=""

while [ $# -gt 0 ]; do
    case "$1" in
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
        --zed-ai)
            zed=true
            zed_ai=true
            shift  # Enables ZED AI SDK
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

        # group configuration
        --config-groups)
            config_groups=true
            shift  # Configures user groups associated with hardware
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

        # mj-source-dir
        --mj-source-dir)
            if [ -n "$2" ] && [ "${2:0:1}" != "-" ]; then
                mj_source_dir="$2"
                shift 2
            else
                echo "Error: --mj-source-dir requires a directory path as an argument."
                exit 1
            fi
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
  --zed-ai                     Enables ZED SDK with AI Module

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --config-groups              Configures user groups associated with hardware

  --install-sys-deps           Installs system dependencies
  --source-ros                 Sources base ROS in ~/.bashrc (only used if --install-sys-deps)

  --pixi                       Install pixi
  --obk-aliases                Add obelisk aliases to the ~/.bash_aliases file

  --mj-source-dir <path>       Specify the source directory for MuJoCo

  --help                       Display this help message and exit
"
            shift
            exit
            ;;
        *)
            # Unknown option
            echo "Unknown option: $1. Run 'source setup.sh --help' for more information."
            exit 1
            ;;
    esac
done

# set OBELISK_ROOT to be the directory containing this script
export OBELISK_ROOT=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# docker setup
# if you want to use the ZED camera, you always need the local deps
# always set groups in docker for hardware, regardless of whether you're installing the deps
if [ "$install_sys_deps_docker" = true ]; then
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$basic" = true ] && echo "--docker-basic") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$leap" = true ] && echo "--docker-leap --docker-group-leap") \
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$zed_ai" = true ] && echo "--docker-zed --docker-zed-ai --docker-group-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi") \
        $([ -n "$mj_source_dir" ] && echo "--docker-mj-source-dir $mj_source_dir")
else
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$leap" = true ] && echo "--docker-group-leap") \
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$zed_ai" = true ] && echo "--docker-zed-ai --docker-group-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi") \
        $([ -n "$mj_source_dir" ] && echo "--docker-mj-source-dir $mj_source_dir")
fi

# group configuration on host
if [ "$config_groups" = true ]; then
    source $OBELISK_ROOT/scripts/config_groups.sh \
        $([ "$leap" = true ] && echo "--leap") \
        $([ "$zed" = true ] && echo "--zed")
fi

# system-level deps
if [ "$install_sys_deps" = true ]; then
    source $OBELISK_ROOT/scripts/install_sys_deps.sh \
        $([ "$basic" = true ] && echo "--basic") \
        $([ "$cyclone_perf" = true ] && echo "--cyclone-perf") \
        $([ "$source_ros" = true ] && echo "--source-ros") \
        $([ "$leap" = true ] && echo "--leap") \
        $([ "$zed" = true ] && echo "--zed") \
        $([ "$zed_ai" = true ] && echo "--zed-ai")
fi

# run user-specific setup
source $OBELISK_ROOT/scripts/user_setup.sh \
    $([ "$pixi" = true ] && echo "--pixi") \
    $([ "$leap" = true ] && echo "--leap") \
    $([ "$zed" = true ] && echo "--zed") \
    $([ "$obk_aliases" = true ] && echo "--obk-aliases") \
    $([ -n "$mj_source_dir" ] && echo "--mj-source-dir $mj_source_dir")

# if using the zed flag, create a persistent named docker volume for the zed folder if docker is installed
if command -v docker > /dev/null 2>&1 && [ "$zed_ai" = true ] && [ ! "$(docker volume ls -q -f name=zed)" ]; then
    echo -e "\033[1;32mCreating persistent named volume for ZED AI SDK!\033[0m"
    docker volume create zed
fi
