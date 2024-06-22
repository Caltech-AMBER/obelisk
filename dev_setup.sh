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

# create a .env file under the docker directory with the USER, UID, and GID of the local system
if [ ! -f "$OBELISK_ROOT/docker/.env" ]; then
	echo "USER=$USER" > $OBELISK_ROOT/docker/.env
	echo "UID=$(id -u)" >> $OBELISK_ROOT/docker/.env
	echo "GID=$(id -g)" >> $OBELISK_ROOT/docker/.env
	echo -e "\033[1;32m.env file created under $OBELISK_ROOT/docker!\033[0m"
else
	echo -e "\033[1;33m.env file already exists under $OBELISK_ROOT/docker, skipping...\033[0m"
fi
