#!/bin/bash

# basic dependencies
sudo apt-get install -y \
	git

# docker
# also, see https://stackoverflow.com/questions/48957195/how-to-fix-docker-got-permission-denied-issue
if ! command -v docker &> /dev/null; then
    echo -e "\033[1;32mDocker is not installed. Installing Docker...\033[0m"

	sudo apt-get update
	sudo apt-get install ca-certificates curl
	sudo install -m 0755 -d /etc/apt/keyrings
	sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
	sudo chmod a+r /etc/apt/keyrings/docker.asc
	echo \
	"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
	$(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
	sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
	sudo apt-get update
	sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

	echo -e "\033[1;32mDocker setup complete. Continuing with the script.\033[0m"
else
    echo -e "\033[1;33mDocker is already installed. Skipping Docker installation.\033[0m"
fi

# check docker group exists
if ! getent group docker &> /dev/null; then
	echo -e "\033[1;32mDocker group does not exist. Creating Docker group and adding user...\033[0m"

	sudo groupadd docker
	sudo usermod -aG docker $USER
	sg docker -c 'sudo chmod 660 /var/run/docker.sock && sudo systemctl restart docker'
else
	echo -e "\033[1;33mDocker group already exists. Skipping Docker group creation.\033[0m"
fi

# if the system has nvidia GPUs, install nvidia-container-toolkit
is_nvidia_container_toolkit_installed() {
    dpkg -s nvidia-container-toolkit &> /dev/null
}
if ! is_nvidia_container_toolkit_installed; then
    echo -e "\033[1;32mNVIDIA Container Toolkit is not installed. Installing...\033[0m"
	if (($(nvidia-smi -L | wc -l) > 0)); then
		curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
		&& curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
			sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
			sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
		sudo apt-get update
		sudo apt-get install -y nvidia-container-toolkit
		sudo nvidia-ctk runtime configure --runtime=docker
		sudo systemctl restart docker
		echo -e "\033[1;32mNVIDIA Container Toolkit installation complete!\033[0m"
    else
        echo -e "\033[1;33mNVIDIA GPU not detected. Skipping NVIDIA Container Toolkit installation.\033[0m"
    fi
else
    echo -e "\033[1;33mNVIDIA Container Toolkit is already installed. Skipping installation.\033[0m"
fi

# installing pixi
if ! command -v pixi &> /dev/null; then
	echo -e "\033[1;32mPixi is not installed. Installing Pixi...\033[0m"
	curl -fsSL https://pixi.sh/install.sh | bash
else
	echo -e "\033[1;33mPixi is already installed. Skipping Pixi installation.\033[0m"
fi

# installing uv
if ! command -v uv &> /dev/null; then
	echo -e "\033[1;32muv is not installed. Installing uv...\033[0m"
	curl -LsSf https://astral.sh/uv/install.sh | sh
else
	echo -e "\033[1;33muv is already installed. Skipping uv installation.\033[0m"
fi

# installing nvm
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
if ! command -v nvm &> /dev/null; then
	echo -e "\033[1;32mnvm is not installed. Installing nvm...\033[0m"
	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
	export NVM_DIR="$HOME/.nvm"
	[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
	[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
	source ~/.bashrc
	nvm install 20
else
	echo -e "\033[1;33mnvm is already installed. Skipping NVM installation.\033[0m"
fi

# set OBELISK_ROOT to the directory where dev_setup.sh is located if it doesn't exist already
if [ -z "$OBELISK_ROOT" ]; then
	export OBELISK_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	echo "export OBELISK_ROOT=$OBELISK_ROOT" >> ~/.bashrc
	echo -e "\033[1;32mOBELISK_ROOT is now set to $OBELISK_ROOT!\033[0m"
else
	echo -e "\033[1;33mOBELISK_ROOT is already set to $OBELISK_ROOT, skipping...\033[0m"
fi

# set PYRIGHT_PYTHON_FORCE_VERSION=latest and add to .bashrc
if ! grep -q "PYRIGHT_PYTHON_FORCE_VERSION=latest" ~/.bashrc; then
	echo "export PYRIGHT_PYTHON_FORCE_VERSION=latest" >> ~/.bashrc
	echo -e "\033[1;32mPYRIGHT_PYTHON_FORCE_VERSION=latest added to ~/.bashrc!\033[0m"
else
	echo -e "\033[1;33mPYRIGHT_PYTHON_FORCE_VERSION=latest already exists in ~/.bashrc, skipping...\033[0m"
fi

# create a .env file under the docker directory with the USER, UID, and GID of the local system
if [ ! -f "$OBELISK_ROOT/docker/.env" ]; then
	echo "USER=$USER" > $OBELISK_ROOT/docker/.env
	echo "UID=$(id -u)" >> $OBELISK_ROOT/docker/.env
	echo "GID=$(id -g)" >> $OBELISK_ROOT/docker/.env
	echo "OBELISK_ROOT=$OBELISK_ROOT" >> $OBELISK_ROOT/docker/.env
	echo -e "\033[1;32m.env file created under $OBELISK_ROOT/docker!\033[0m"
else
	echo -e "\033[1;33m.env file already exists under $OBELISK_ROOT/docker, skipping...\033[0m"
fi

# colorized error messages for ROS2
if ! grep -q "export RCUTILS_COLORIZED_OUTPUT=1" ~/.bashrc; then
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
    echo -e "\033[1;32mRCUTILS_COLORIZED_OUTPUT=1 added to ~/.bashrc!\033[0m"
else
    echo -e "\033[1;33mRCUTILS_COLORIZED_OUTPUT=1 already exists in ~/.bashrc, skipping...\033[0m"
fi

# add some obelisk aliases to the .bashrc
obk_aliases=$(cat << 'EOF'
# >>> obelisk >>>
# !! Contents in this block are managed by obelisk !!

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

    ros2 launch obelisk_ros obelisk_bringup.launch.py config_file_path:=${config_file_path} device_name:=${device_name} auto_start:=${auto_start}
}

# help command
alias obk-help='echo -e "\033[1;34mObelisk Commands:\n\
obk-launch:\n\
  Launches the obelisk_bringup.launch.py with specified arguments.\n\
  Usage: obk-launch config_file_path=<path> device_name=<name> auto_start=<True|False>\n\
  Example:\n  obk-launch config_file_path=example.yaml device_name=onboard auto_start=True\n\n\
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
sed -i '/# >>> obelisk >>>/,/# <<< obelisk <<</d' ~/.bashrc
echo "$obk_aliases" >> ~/.bashrc
echo -e "\033[1;32mObelisk aliases added to ~/.bashrc!\033[0m"
