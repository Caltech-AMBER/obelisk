#!/bin/bash

# script flags
basic=false
cyclone_perf=false
leap=false
zed=false
unitree=false
fr3=false

docker_install=false
install_sys_deps_docker=false

config_groups=false

install_sys_deps=false
source_ros=false

pixi=false
obk_aliases=false

for arg in "$@"; do
    case $arg in
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
        --unitree)
            unitree=true
            shift
            ;;
        --fr3)
            fr3=true
            shift  # Enables FR3 dependencies
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

        # hardware credentials
        --fr3-username=*)
            FR3_USERNAME="${arg#*=}"
            export FR3_USERNAME
            ;;
        --fr3-password=*)
            FR3_PASSWORD="${arg#*=}"
            export FR3_PASSWORD
            ;;

        # help
        --help)
            echo "Usage: source setup.sh [OPTIONS]

Options:
  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --leap                       Enables LEAP hand dependencies
  --zed                        Enables ZED SDK
  --unitree                    Enables the unitree interfaces
  --fr3                        Enables FR3 dependencies
  --fr3-username=[USERNAME]    Set the FR3 username
  --fr3-password=[PASSWORD]    Set the FR3 password

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --config-groups              Configures user groups associated with hardware

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
# always set groups in docker for hardware, regardless of whether you're installing the deps
if [ "$install_sys_deps_docker" = true ]; then
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$basic" = true ] && echo "--docker-basic") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$leap" = true ] && echo "--docker-leap --docker-group-leap") \
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi") \
        $([ "$unitree" = true ] && echo "--docker-unitree") \
        $([ "$fr3" = true ] && echo "--docker-fr3") \
        $([ -n "$FR3_USERNAME" ] && echo "--docker-fr3-username=$FR3_USERNAME") \
        $([ -n "$FR3_PASSWORD" ] && echo "--docker-fr3-password=$FR3_PASSWORD")
else
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$leap" = true ] && echo "--docker-group-leap") \
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$pixi" = true ] && echo "--docker-pixi") \
        $([ "$unitree" = true ] && echo "--docker-unitree") \
        $([ "$fr3" = true ] && echo "--docker-fr3") \
        $([ -n "$FR3_USERNAME" ] && echo "--docker-fr3-username=$FR3_USERNAME") \
        $([ -n "$FR3_PASSWORD" ] && echo "--docker-fr3-password=$FR3_PASSWORD")
fi

# group configuration on host
if [ "$config_groups" = true ]; then
    source $OBELISK_ROOT/scripts/config_groups.sh \
        $([ "$leap" = true ] && echo "--leap") \
        $([ "$zed" = true ] && echo "--zed") \
        $([ "$fr3" = true ] && echo "--fr3")
fi

# system-level deps
if [ "$install_sys_deps" = true ]; then
    source $OBELISK_ROOT/scripts/install_sys_deps.sh \
        $([ "$basic" = true ] && echo "--basic") \
        $([ "$cyclone_perf" = true ] && echo "--cyclone-perf") \
        $([ "$source_ros" = true ] && echo "--source-ros") \
        $([ "$leap" = true ] && echo "--leap") \
        $([ "$zed" = true ] && echo "--zed") \
        $([ "$unitree" = true ] && echo "--unitree") \
        $([ "$fr3" = true ] && echo "--fr3")
fi

# run user-specific setup
source $OBELISK_ROOT/scripts/user_setup.sh \
    $([ "$pixi" = true ] && echo "--pixi") \
    $([ "$leap" = true ] && echo "--leap") \
    $([ "$zed" = true ] && echo "--zed") \
    $([ "$unitree" = true ] && echo "--unitree") \
    $([ "$fr3" = true ] && echo "--fr3") \
    $([ "$obk_aliases" = true ] && echo "--obk-aliases")
