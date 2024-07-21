#!/bin/bash

# installing pixi
if ! command -v pixi &> /dev/null; then
	echo -e "\033[1;32mPixi is not installed. Installing Pixi...\033[0m"
	curl -fsSL https://pixi.sh/install.sh | bash
else
	echo -e "\033[1;33mPixi is already installed. Skipping Pixi installation.\033[0m"
fi

# add some obelisk aliases to the ~/.bash_aliases file
obk_aliases=$(cat << 'EOF'
# >>> obelisk >>>
# !! Contents in this block are managed by obelisk !!

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
alias obk-help='echo -e "\033[1;34mObelisk Commands:\n\
obk-build:\n\
  Builds Obelisk nodes after you have activated a pixi environment.\n\
  Usage:
    pixi shell -e <env_name>
    obk-build\n\n\
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
