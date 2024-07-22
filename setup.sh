#!/bin/bash

# script flags
skip_docker=true
cyclone_perf=false
pixi=false
bash_aliases=false
obk_aliases=false

for arg in "$@"; do
    case $arg in
        --all)
            skip_docker=false
            cyclone_perf=true
            pixi=true
            bash_aliases=true
            obk_aliases=true
            shift # Allows all system-level changes at once
            ;;
        --no-skip-docker)
            skip_docker=false
            shift # Enables Docker and nvidia-container-toolkit installation
            ;;
        --cyclone-perf)
            cyclone_perf=true
            shift # Enables cyclone performance optimizations
            ;;
        --pixi)
            pixi=true
            shift # Installs pixi
            ;;
        --bash-aliases)
            bash_aliases=true
            shift # Ensures the ~/.bash_aliases file is created and sourced in ~/.bashrc
            ;;
        --obk-aliases)
            obk_aliases=true
            shift # Adds obelisk aliases to the ~/.bash_aliases file
            ;;
        --help)
            echo "Usage: source setup.sh [--no-skip-docker] [--pixi] [--cyclone-perf] [--bash-aliases] [--obk-aliases]"
            return
            ;;
        -h)
            echo "Usage: source setup.sh [--no-skip-docker] [--pixi] [--cyclone-perf] [--bash-aliases] [--obk-aliases]"
            return
            ;;
        *)
            # Unknown option
            echo "Unknown option: $arg"
            echo "Usage: source setup.sh [--no-skip-docker] [--pixi] [--cyclone-perf] [--bash-aliases] [--obk-aliases]"
            exit 1
            ;;
    esac
done

# [1] installs docker and nvidia-container-toolkit
if [ "$skip_docker" = true ]; then
    echo -e "\033[1;33mSkipping Docker and nvidia-container-toolkit installation.\033[0m"
else
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
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
                sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
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
fi

# [2] create a .env file under the docker directory with the USER, UID, GID of the local system + OBELISK_ROOT
# create or delete and replace the contents of $OBELISK_ROOT/docker/.env
export OBELISK_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
env_file="$OBELISK_ROOT/docker/.env"
if [ -f $env_file ]; then
    rm $env_file
fi
touch $env_file
echo "USER=$USER" > $env_file
echo "UID=$(id -u)" >> $env_file
echo "GID=$(id -g)" >> $env_file
echo "OBELISK_ROOT=$OBELISK_ROOT" >> $env_file
echo -e "\033[1;32m.env file populated under $OBELISK_ROOT/docker!\033[0m"

# rest of setup commands from docker/docker_setup.sh
source docker/docker_setup.sh \
    $([ "$pixi" = true ] && echo "--pixi") \
    $([ "$cyclone_perf" = true ] && echo "--cyclone-perf") \
    $([ "$bash_aliases" = true ] && echo "--bash-aliases") \
    $([ "$obk_aliases" = true ] && echo "--obk-aliases")
