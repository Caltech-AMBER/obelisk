#!/bin/bash

# script flags
basic=false
cyclone_perf=false
zed=false
unitree=false

docker_install=false
install_sys_deps_docker=false

config_groups=false

install_sys_deps=false
source_ros=false

obk_aliases=false

for arg in "$@"; do
    case $arg in
        --dev-setup)
            obk_aliases=true
            cyclone_perf=true
            docker_install=true
            install_sys_deps_docker=true
            basic=true
            shift  # Setup for development of Obelisk.
            ;;
        
        --downstream-setup)
            obk_aliases=true
            cyclone_perf=true
            basic=true
            install_sys_deps=true
            shift  # Setup for downstream use of Obelisk - suggested to build this in a docker container.
            ;;

        # --recommended)
        #     cyclone_perf=true
        #     obk_aliases=true
        #     shift  # Allows recommended system-level changes
        #     ;;

        # # dependency groups to configure
        # --basic)
        #     basic=true
        #     shift  # Enables basic dependencies necessary for Obelisk locally
        #     ;;
        # --cyclone-perf)
        #     cyclone_perf=true
        #     shift # Enables cyclone performance optimizations
        #     ;;

        # Hardware options
        --zed)
            zed=true
            shift  # Enables ZED SDK
            ;;
        --unitree)
            unitree=true
            shift
            ;;


        # # docker setup
        # --docker-install)
        #     docker_install=true
        #     shift  # Installs Docker and nvidia-container-toolkit
        #     ;;
        # --install-sys-deps-docker)
        #     install_sys_deps_docker=true
        #     shift  # Installs system dependencies in Docker
        #     ;;

        # # group configuration
        # --config-groups)
        #     config_groups=true
        #     shift  # Configures user groups associated with hardware
        #     ;;

        # # system-level deps
        # --install-sys-deps)
        #     install_sys_deps=true
        #     shift  # Installs system dependencies
        #     ;;
        # --source-ros)
        #     source_ros=true
        #     shift # Sources base ROS in ~/.bashrc (only used if --install-sys-deps)
        #     ;;

        # # user setup
        # --obk-aliases)
        #     obk_aliases=true
        #     shift # Adds obelisk aliases to the ~/.bash_aliases file
        #     ;;

        # help
        --help)
            echo "Usage: source setup.sh [OPTIONS]

Options:
  --dev-setup                  Setup for development of Obelisk.

  --downstream-setup           Setup for downstream use of Obelisk - suggested to build this in a docker container.

  Hardware options:
  --zed                        Enables ZED SDK
  --unitree                    Enables the unitree interfaces

  Other options:
  --help                       Display this help message and exit


  ===== DEPRECATED OPTIONS =====
  --recommended                Apply recommended system-level changes
                               (cyclone performance optimizations, pixi, obelisk aliases)

  --basic                      Enables basic dependencies necessary for Obelisk locally
  --cyclone-perf               Enables cyclone performance optimizations
  --zed                        Enables ZED SDK
  --unitree                    Enables the unitree interfaces

  --docker-install             Install Docker and nvidia-container-toolkit
  --install-sys-deps-docker    Installs system dependencies in Docker

  --config-groups              Configures user groups associated with hardware

  --install-sys-deps           Installs system dependencies
  --source-ros                 Sources base ROS in ~/.bashrc (only used if --install-sys-deps)

  --obk-aliases                Add obelisk aliases to the ~/.bash_aliases file

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
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$unitree" = true ] && echo "--docker-unitree")
else
    source $OBELISK_ROOT/scripts/docker_setup.sh \
        $([ "$docker_install" = true ] && echo "--docker-install") \
        $([ "$cyclone_perf" = true ] && echo "--docker-cyclone-perf") \
        $([ "$zed" = true ] && echo "--docker-zed --docker-group-zed") \
        $([ "$unitree" = true ] && echo "--docker-unitree")
fi

# group configuration on host
if [ "$config_groups" = true ]; then
    source $OBELISK_ROOT/scripts/config_groups.sh \
        $([ "$zed" = true ] && echo "--zed")
fi

# system-level deps
if [ "$install_sys_deps" = true ]; then
    source $OBELISK_ROOT/scripts/install_sys_deps.sh \
        $([ "$basic" = true ] && echo "--basic") \
        $([ "$cyclone_perf" = true ] && echo "--cyclone-perf") \
        $([ "$source_ros" = true ] && echo "--source-ros") \
        $([ "$zed" = true ] && echo "--zed") \
        $([ "$unitree" = true ] && echo "--unitree")
fi

# run user-specific setup
source $OBELISK_ROOT/scripts/user_setup.sh \
    $([ "$pixi" = true ] && echo "--pixi") \
    $([ "$zed" = true ] && echo "--zed") \
    $([ "$unitree" = true ] && echo "--unitree") \
    $([ "$obk_aliases" = true ] && echo "--obk-aliases")
