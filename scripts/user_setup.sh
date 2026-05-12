#!/bin/bash

# script flags
obk_aliases=false

zed=false
unitree=false
westwood=false

for arg in "$@"; do
    case $arg in
        # general user setup
        --obk-aliases)
            obk_aliases=true
            shift  # Adds obelisk aliases to the ~/.bash_aliases file
            ;;

        # alias configuration
        --zed)
            zed=true
            shift  # Adds ZED ROS packages to colcon build command
            ;;
        --unitree)
            unitree=true
            shift  # Sets up the Unitree interface
            ;;
        --westwood)
            westwood=true
            shift  # Sets up the Westwood THEMIS interface
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--obk-aliases] [--zed] [--unitree] [--westwood]"
            exit 1
            ;;
    esac
done

# adds obelisk aliases to the ~/.bash_aliases file
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
    if [ "$zed" = true ]; then
        OBELISK_BUILD_OPTIONS+="--zed "
        OBELISK_BUILD_ZED=true
    else
        OBELISK_BUILD_ZED=false
    fi
    if [ "$unitree" = true ]; then
        OBELISK_BUILD_UNITREE=true
        ROS_DOMAIN_ID=2
        echo -e "\033[1;32mSetting ROS_DOMAIN_ID=2\033[0m"
    else
        OBELISK_BUILD_UNITREE=false
    fi
    if [ "$westwood" = true ]; then
        OBELISK_BUILD_WESTWOOD=true
    else
        OBELISK_BUILD_WESTWOOD=false
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
export OBELISK_BUILD_ZED=$OBELISK_BUILD_ZED
export OBELISK_BUILD_UNITREE=$OBELISK_BUILD_UNITREE
export OBELISK_BUILD_WESTWOOD=$OBELISK_BUILD_WESTWOOD
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
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

    # checks if current shell is a conda shell - if not, source base ros if /opt/ros/humble/setup.bash exists
    # in the case of conda, it will source base ros if \$CONDA_DEFAULT_ENV is "base" still
    if [[ -z "\$CONDA_DEFAULT_ENV" || "\$CONDA_DEFAULT_ENV" == "base" ]]; then
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
    local config=""
    local auto_start="True"
    local bag="False"

    while [[ \$# -gt 0 ]]; do
        key="\$1"
        case \$key in
            config=*)
            config="\${key#*=}"
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

    if [[ -z "\$config" ]]; then
        echo -e "\033[1;34mError: Missing required argument.\033[0m"
        echo -e "\033[1;34mUsage: obk-launch config=<path> [auto_start=<True|False>] [bag=<True|False>]\033[0m"
        return 1
    fi

    ros2 launch obelisk_ros obelisk_bringup.launch.py config_file_path:=\${config} auto_start:=\${auto_start} bag:=\${bag}
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
Builds Obelisk ROS2 nodes via colcon (run inside the dev container).\n\n\
obk-launch:\n\
Launches the obelisk_bringup.launch.py with specified arguments.\n\
Usage: obk-launch config=<path> [auto_start=<True|False>] [bag=<True|False>]\n\
Defaults: auto_start=True, bag=False (pass bag=True to record a rosbag of all topics).\n\
Example:\n  obk-launch config=example.yaml\n\n\
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
