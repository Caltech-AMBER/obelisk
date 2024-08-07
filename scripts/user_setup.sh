#!/bin/bash

# script flags
pixi=false
obk_aliases=false

leap=false
zed=false

for arg in "$@"; do
    case $arg in
        # general user setup
        --pixi)
            pixi=true
            shift  # Installs pixi
            ;;
        --obk-aliases)
            obk_aliases=true
            shift  # Adds obelisk aliases to the ~/.bash_aliases file
            ;;

        # alias configuration
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
            echo "Usage: $0 [--cyclone-perf] [--obk-aliases]"
            exit 1
            ;;
    esac
done

# [1] installs pixi
if [ "$pixi" = true ]; then
    if ! command -v pixi &> /dev/null; then
        echo -e "\033[1;32mPixi is not installed. Installing Pixi...\033[0m"
        curl -fsSL https://pixi.sh/install.sh | bash
    else
        echo -e "\033[1;33mPixi is already installed. Skipping Pixi installation.\033[0m"
    fi
fi

# [2] adds obelisk aliases to the ~/.bash_aliases file
if [ "$obk_aliases" = true ]; then
    # check if ~/.bash_aliases is sourced in ~/.bashrc; if not, add it
    block='if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi'
    if ! grep -q "$block" ~/.bashrc; then
        echo "$block" >> ~/.bashrc
        echo -e "\033[1;32mAdded code to source ~/.bash_aliases in ~/.bashrc!\033[0m"
    fi

    # check whether ~/.bash_aliases exists; if not, touch it
    if [ ! -f ~/.bash_aliases ]; then
        touch ~/.bash_aliases
        echo -e "\033[1;32mCreated ~/.bash_aliases file!\033[0m"
    else
        echo -e "\033[1;33m~/.bash_aliases file already exists, not creating a new one...\033[0m"
    fi

    # creating obelisk aliases
    OBELISK_ROOT=$(dirname $(dirname $(readlink -f ${BASH_SOURCE[0]})))
    OBELISK_BUILD_OPTIONS=""
    if [ "$leap" = true ]; then
        OBELISK_BUILD_OPTIONS+="--leap "
        OBELISK_BUILD_LEAP=true
    else
        OBELISK_BUILD_LEAP=false
    fi
    if [ "$zed" = true ]; then
        OBELISK_BUILD_OPTIONS+="--zed "
        OBELISK_BUILD_ZED=true
    else
        OBELISK_BUILD_ZED=false
    fi

    obk_aliases=$(cat << EOF
# >>> obelisk >>>
# !! Contents in this block are managed by obelisk !!

# alias for setting up obelisk global settings in current shell
function obk {
    cmd='
export OBELISK_ROOT=\$OBELISK_ROOT
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_COLORIZED_OUTPUT=1
export OBELISK_BUILD_LEAP=$OBELISK_BUILD_LEAP
export OBELISK_BUILD_ZED=$OBELISK_BUILD_ZED
'

    # Check if the --permanent flag is passed
    if [[ "\$1" == "--permanent" ]]; then
        sed '/# >>> obk >>>/,/# <<< obk <<</d' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
        echo "# >>> obk >>>" >> ~/.bashrc
        echo "\$cmd" >> ~/.bashrc
        echo "# <<< obk <<<" >> ~/.bashrc
        echo -e "\033[1;32mObelisk global settings added to ~/.bashrc!\033[0m"
    fi

    # Check if the --remove flag is passed
    if [[ "\$1" == "--remove" ]]; then
        sed '/# >>> obk >>>/,/# <<< obk <<</d' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
        echo -e "\033[1;32mObelisk global settings removed from ~/.bashrc!\033[0m"
        return
    fi

    if [[ -z "\$HAS_OBK_ACTIVATED" ]]; then
        # globally useful obelisk settings
        eval "\$cmd"

        # edits the shell prompt to include [obk] the first time obk is called in this shell
        echo -e "\033[1;32mObelisk global settings applied!\033[0m"
        BLUE="\[\033[0;34m\]"
        RESET="\[\033[0m\]"
        PS1="\${BLUE}[obk]\${RESET} \$PS1"
        export PS1
        export HAS_OBK_ACTIVATED=true
    fi

    # checks if current shell is a conda or pixi shell - if not, source base ros if /opt/ros/humble/setup.bash exists
    # in the case of conda, it will source base ros if \$CONDA_DEFAULT_ENV is "base" still
    if [[ -z "\$CONDA_DEFAULT_ENV" || "\$CONDA_DEFAULT_ENV" == "base" ]] && [[ -z "\$PIXI_ENVIRONMENT_NAME" ]]; then
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            echo -e "\033[1;32mSourcing base ROS2 Humble installation...\033[0m"
            source /opt/ros/humble/setup.bash
        fi
    fi

    # regardless of whether obk has been run, source the obelisk ros packages
    if [ -d "\$OBELISK_ROOT/obelisk_ws/install" ]; then
        source \$OBELISK_ROOT/obelisk_ws/install/setup.bash
    else
        source \$OBELISK_ROOT/scripts/build_obelisk.sh $OBELISK_BUILD_OPTIONS
        source \$OBELISK_ROOT/obelisk_ws/install/setup.bash
    fi

    # if PIXI_ZED=true, then run bash $OBELISK_ROOT/scripts/install_pyzed.sh
    if [[ -n "\$PIXI_ZED" && "\$PIXI_ZED" == "true" ]]; then
        bash \$OBELISK_ROOT/scripts/install_pyzed.sh
    fi
}

# convenience aliases for building/cleaning obelisk source packages
alias obk-build='source \$OBELISK_ROOT/scripts/build_obelisk.sh $OBELISK_BUILD_OPTIONS'
alias obk-clean='bash \$OBELISK_ROOT/scripts/clean_obelisk.sh'

# convenience aliases for lifecycle commands
function obk-lifecycle {
    if [[ -z "\$1" || -z "\$2" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-lifecycle <node> <state>\033[0m"
        return 1
    fi
    ros2 lifecycle set "\$1" "\$2"
}

function obk-configure {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-configure <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" configure
}

function obk-activate {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-activate <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" activate
}

function obk-deactivate {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-deactivate <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" deactivate
}

function obk-cleanup {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-cleanup <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" cleanup
}

function obk-shutdown {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-shutdown <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" shutdown
}

function obk-start {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-start <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" configure
}

function obk-stop {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-stop <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" deactivate
    obk-lifecycle "\$1" cleanup
}

function obk-kill {
    if [[ -z "\$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-kill <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "\$1" shutdown
}

# convenience function for ros2 launch command
function obk-launch {
    local config_file_path=""
    local device_name=""
    local auto_start="True"
    local bag="True"

    while [[ \$# -gt 0 ]]; do
        key="\$1"
        case \$key in
            config_file_path=*)
            config_file_path="\${key#*=}"
            shift
            ;;
            device_name=*)
            device_name="\${key#*=}"
            shift
            ;;
            auto_start=*)
            auto_start="\${key#*=}"
            shift
            ;;
            bag=*)
            bag="\${key#*=}"
            shift
            ;;
            *)
            echo "Unknown option \$key"
            return 1
            ;;
        esac
    done

    # Check if any of the required arguments are empty
    if [[ -z "\$config_file_path" || -z "\$device_name" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-launch config_file_path=<path> device_name=<name> auto_start=<True|False>\033[0m"
        return 1
    fi

    ros2 launch obelisk_ros obelisk_bringup.launch.py config_file_path:=\${config_file_path} device_name:=\${device_name} auto_start:=\${auto_start} bag:=\${bag}
}

# help command
alias obk-help='echo -e "\033[1;34m================\n\
Obelisk Commands\n\
================\n\
obk:\n\
Sets up global environment variables for Obelisk in the current shell.\n\
Usage: obk [--permanent|--remove]\n\
Options:\n\
    --permanent: Adds the global settings to ~/.bashrc.\n\
    --remove: Removes the global settings from ~/.bashrc.\n\n\
obk-build:\n\
Builds Obelisk nodes after you have activated a pixi environment.\n\n\
obk-launch:\n\
Launches the obelisk_bringup.launch.py with specified arguments.\n\
Usage: obk-launch config_file_path=<path> device_name=<name> auto_start=<True|False> bag=<True|False>\n\
Example:\n  obk-launch config_file_path=example.yaml device_name=onboard auto_start=True bag=True\n\n\
State Transitions:\n\
obk-configure:\n    Configure all Obelisk nodes.\n    Usage: obk-configure <config_name>\n\
obk-activate:\n    Activate all Obelisk nodes.\n    Usage: obk-activate <config_name>\n\
obk-deactivate:\n    Deactivate all Obelisk nodes.\n    Usage: obk-deactivate <config_name>\n\
obk-cleanup:\n    Cleanup all Obelisk nodes.\n    Usage: obk-cleanup <config_name>\n\
obk-shutdown:\n    Shutdown all Obelisk nodes.\n    Usage: obk-shutdown <config_name>\n\n\
Convenience Commands:\n\
obk-start:\n    Alias for obk-configure.\n    Usage: obk-start <config_name>\n\
obk-stop:\n    Alias for obk-deactivate and obk-cleanup.\n    Usage: obk-stop <config_name>\n\
obk-kill:\n    Alias for obk-shutdown.\n    Usage: obk-kill <config_name>\n\n\
obk-help:\n    Display this help message.\n\
In all the above commands, <config_name> refers to the config field of the config file used for launching obelisk.\
\033[0m"'
# <<< obelisk <<<
EOF
)
    sed '/# >>> obelisk >>>/,/# <<< obelisk <<</d' ~/.bash_aliases > ~/.bash_aliases.tmp && cp ~/.bash_aliases.tmp ~/.bash_aliases && rm ~/.bash_aliases.tmp
    echo "$obk_aliases" >> ~/.bash_aliases
    echo -e "\033[1;32mObelisk aliases added to ~/.bash_aliases!\033[0m"
else
    echo -e "\033[1;33mObelisk aliases not added to ~/.bash_aliases. To add, pass the --obk-aliases flag.\033[0m"
fi

# Only source if the file exists. In the docker build this file does not exists and does not need to be run, so we can skip it.
if test -f ${OBELISK_ROOT}/scripts/_source_bash_rc.bash; then
    source ${OBELISK_ROOT}/scripts/_source_bash_rc.bash
fi
