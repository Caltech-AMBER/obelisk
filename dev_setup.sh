#!/bin/bash

# basic dependencies
sudo apt-get install -y \
	git

# docker
# also, see https://stackoverflow.com/questions/48957195/how-to-fix-docker-got-permission-denied-issue
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Installing Docker..."

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

	echo "Docker setup complete. Continuing with the script."
else
    echo "Docker is already installed. Skipping Docker installation."
fi

# check docker group exists
if ! getent group docker &> /dev/null; then
	echo "Docker group does not exist. Creating Docker group and adding user..."

	sudo groupadd docker
	sudo usermod -aG docker $USER
	sg docker -c 'sudo chmod 660 /var/run/docker.sock && sudo systemctl restart docker'
else
	echo "Docker group already exists. Skipping Docker group creation."
fi

# if the system has nvidia GPUs, install nvidia-container-toolkit
is_nvidia_container_toolkit_installed() {
    dpkg -s nvidia-container-toolkit &> /dev/null
}
if ! is_nvidia_container_toolkit_installed; then
    echo "NVIDIA Container Toolkit is not installed. Installing..."
	if (($(nvidia-smi -L | wc -l) > 0)); then
		curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg -y && \
			curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
			sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
			sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
		sudo apt-get update
		sudo apt-get install -y nvidia-container-toolkit
		sudo nvidia-ctk runtime configure --runtime=docker
		sudo systemctl restart docker
		echo "NVIDIA Container Toolkit installation complete."
    else
        echo "NVIDIA GPU not detected. Skipping NVIDIA Container Toolkit installation."
    fi
else
    echo "NVIDIA Container Toolkit is already installed. Skipping installation."
fi

# installing pixi
if ! command -v pixi &> /dev/null; then
	echo "Pixi is not installed. Installing Pixi..."
	curl -fsSL https://pixi.sh/install.sh | bash
else
	echo "Pixi is already installed. Skipping Pixi installation."
fi

# installing nvm
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
if ! command -v nvm &> /dev/null; then
	echo "nvm is not installed. Installing nvm..."
	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
	export NVM_DIR="$HOME/.nvm"
	[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
	[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
	source ~/.bashrc
	nvm install 20
else
	echo "nvm is already installed. Skipping NVM installation."
fi

# TODO(ahl): replace the functionality of newgrp
if ! groups | grep -q "\bdocker\b"; then
	echo "Restarting shell to apply Docker group changes..."
	newgrp docker
else
	echo "Docker group changes already applied. Skipping shell restart."
fi
