#!/bin/bash

# script flags
cyclone_perf=false
obk_aliases=false

for arg in "$@"; do
    case $arg in
        --cyclone-perf)
            cyclone_perf=true
            shift # Enables cyclone performance optimizations
            ;;
        --obk-aliases)
            obk_aliases=true
            shift # Adds obelisk aliases to the ~/.bash_aliases file
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: $0 [--skip-docker]"
            exit 1
            ;;
    esac
done

# [1] installs pixi
if ! command -v pixi &> /dev/null; then
	echo -e "\033[1;32mPixi is not installed. Installing Pixi...\033[0m"
	curl -fsSL https://pixi.sh/install.sh | bash
else
	echo -e "\033[1;33mPixi is already installed. Skipping Pixi installation.\033[0m"
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
    echo -e "\033[1;33mCyclone DDS performance optimizations disabled. To enable, pass the --no-cyclone-perf flag.\033[0m"
fi

# [3] adds obelisk aliases to the ~/.bash_aliases file
if [ "$obk_aliases" = true ]; then
    OBELISK_ROOT=$(dirname $(dirname $(readlink -f ${BASH_SOURCE[0]})))
    obk_aliases=$(cat << 'EOF'
# >>> obelisk >>>
# !! Contents in this block are managed by obelisk !!

# alias for setting up obelisk global settings in current shell
function obk {
    cmd='
export OBELISK_ROOT=$OBELISK_ROOT
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_COLORIZED_OUTPUT=1
'

    # Check if the --permanent flag is passed
    if [[ "$1" == "--permanent" ]]; then
        sed '/# >>> obk >>>/,/# <<< obk <<</d' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
        echo "# >>> obk >>>" >> ~/.bashrc
        echo "$cmd" >> ~/.bashrc
        echo "# <<< obk <<<" >> ~/.bashrc
        echo -e "\033[1;32mObelisk global settings added to ~/.bashrc!\033[0m"
    fi

    # Check if the --remove flag is passed
    if [[ "$1" == "--remove" ]]; then
        sed '/# >>> obk >>>/,/# <<< obk <<</d' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
        echo -e "\033[1;32mObelisk global settings removed from ~/.bashrc!\033[0m"
        return
    fi

    if [[ -z "$HAS_OBK_ACTIVATED" ]]; then
        # globally useful obelisk settings
        eval "$cmd"

        # edits the shell prompt to include [obk] the first time obk is called in this shell
        echo -e "\033[1;32mObelisk global settings applied!\033[0m"
        BLUE="\[\033[0;34m\]"
        RESET="\[\033[0m\]"
        PS1="${BLUE}[obk]${RESET} $PS1"
        export PS1
        export HAS_OBK_ACTIVATED=true
    fi
}

# convenience alias for building obelisk within a pixi env
alias obk-build='source $OBELISK_ROOT/scripts/build_obelisk.sh'

# convenience aliases for lifecycle commands
function obk-lifecycle {
    if [[ -z "$1" || -z "$2" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-lifecycle <node> <state>\033[0m"
        return 1
    fi
    ros2 lifecycle set "$1" "$2"
}

function obk-configure {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-configure <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" configure
}

function obk-activate {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-activate <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" activate
}

function obk-deactivate {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-deactivate <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" deactivate
}

function obk-cleanup {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-cleanup <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" cleanup
}

function obk-shutdown {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-shutdown <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" shutdown
}

function obk-start {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-start <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" configure
}

function obk-stop {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-stop <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" deactivate
    obk-lifecycle "$1" cleanup
}

function obk-kill {
    if [[ -z "$1" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-kill <config_name>\033[0m"
        return 1
    fi
    obk-lifecycle "$1" shutdown
}

# convenience function for ros2 launch command
function obk-launch {
    local config_file_path=""
    local device_name=""
    local auto_start="True"
    local bag="True"

    while [[ $# -gt 0 ]]; do
        key="$1"
        case $key in
            config_file_path=*)
            config_file_path="${key#*=}"
            shift
            ;;
            device_name=*)
            device_name="${key#*=}"
            shift
            ;;
            auto_start=*)
            auto_start="${key#*=}"
            shift
            ;;
            bag=*)
            bag="${key#*=}"
            shift
            ;;
            *)
            echo "Unknown option $key"
            return 1
            ;;
        esac
    done

    # Check if any of the required arguments are empty
    if [[ -z "$config_file_path" || -z "$device_name" ]]; then
        echo -e "\033[1;34mError: Missing required arguments.\033[0m"
        echo -e "\033[1;34mUsage: obk-launch config_file_path=<path> device_name=<name> auto_start=<True|False>\033[0m"
        return 1
    fi

    ros2 launch obelisk_ros obelisk_bringup.launch.py config_file_path:=${config_file_path} device_name:=${device_name} auto_start:=${auto_start} bag:=${bag}
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
