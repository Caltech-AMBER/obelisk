# script flags
docker_install=false

docker_basic=false
docker_cyclone_perf=false
docker_leap=false
docker_zed=false
docker_pixi=false
docker_unitree=false
docker_fr3=false

docker_group_leap=false
docker_group_zed=false
docker_group_fr3=false

for arg in "$@"; do
    case $arg in
        # docker installations
        --docker-install)
            docker_install=true
            shift  # Installs Docker and nvidia-container-toolkit
            ;;
        --docker-basic)
            docker_basic=true
            shift  # Sets OBELISK_DOCKER_BASIC=true for docker, which installs basic dependencies
            ;;
        --docker-cyclone-perf)
            docker_cyclone_perf=true
            shift # Sets OBELISK_DOCKER_CYCLONE_PERF=true for docker, which enables cyclone performance optimizations
            ;;
        --docker-leap)
            docker_leap=true
            shift  # Sets OBELISK_DOCKER_LEAP=true for docker, which installs LEAP hand dependencies
            ;;
        --docker-zed)
            docker_zed=true
            shift  # Sets OBELISK_DOCKER_ZED=true for docker, which installs ZED SDK
            ;;
        --docker-pixi)
            docker_pixi=true
            shift  # Sets OBELISK_DOCKER_PIXI=true for docker, which installs Pixi
            ;;
        --docker-unitree)
            docker_unitree=true
            shift   # Sets OBELISK_DOCKER_UNITREE=true for docker, which configures the docker installation
            ;;
        --docker-fr3)
            docker_fr3=true
            shift  # Sets OBELISK_DOCKER_FR3=true for docker, which enables FR3 dependencies
            ;;

        # docker group additions
        --docker-group-leap)
            docker_group_leap=true
            shift  # Adds user to the dialout group
            ;;
        --docker-group-zed)
            docker_group_zed=true
            shift  # Adds user to the video group
            ;;
        --docker-group-fr3)
            docker_group_fr3=true
            shift  # Adds user to the realtime group
            ;;

        # docker hardware credentials
        --docker-fr3-username=*)
            DOCKER_FR3_USERNAME="${arg#*=}"
            export OBELISK_DOCKER_FR3_USERNAME="$DOCKER_FR3_USERNAME"
            ;;
        --docker-fr3-password=*)
            DOCKER_FR3_PASSWORD="${arg#*=}"
            export OBELISK_DOCKER_FR3_PASSWORD="$DOCKER_FR3_PASSWORD"
            ;;

        --help)
            echo "Usage: $0 [OPTIONS]

Options:
  --docker-install       Install Docker and nvidia-container-toolkit

  --docker-basic         Set OBELISK_DOCKER_BASIC=true for docker, which installs basic dependencies
  --docker-cyclone-perf  Set OBELISK_DOCKER_CYCLONE_PERF=true for docker, which enables cyclone performance optimizations
  --docker-leap          Set OBELISK_DOCKER_LEAP=true for docker, which installs LEAP hand dependencies
  --docker-zed           Set OBELISK_DOCKER_ZED=true for docker, which installs ZED SDK
  --docker-pixi          Set OBELISK_DOCKER_PIXI=true for docker, which installs Pixi
  --docker-unitree       Set OBELISK_DOCKER_UNITREE=true for docker, which configures the docker installation
  --docker-fr3           Set OBELISK_DOCKER_FR3=true for docker, which enables FR3 dependencies

  --docker-group-leap    Adds user to the dialout group
  --docker-group-zed     Adds user to the video group
  --docker-group-fr3     Adds user to the realtime group

  --docker-fr3-username [USERNAME]  Sets the FR3 username for docker
  --docker-fr3-password [PASSWORD]  Sets the FR3 password for docker

  --help                 Display this help message and exit
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

# [1] installs docker and nvidia-container-toolkit
if [ "$docker_install" = true ]; then
    # see https://stackoverflow.com/questions/48957195/how-to-fix-docker-got-permission-denied-issue
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
else
    echo -e "\033[1;33mSkipping Docker and nvidia-container-toolkit installation.\033[0m"
fi

# [2] create a .env file under the docker directory with the USER, UID, GID of the local system + OBELISK_ROOT
# create or delete and replace the contents of $OBELISK_ROOT/docker/.env
export OBELISK_ROOT=$(dirname $(dirname $(readlink -f ${BASH_SOURCE[0]})))

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

# checks additional docker environment variables
if [ "$docker_basic" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_BASIC=true!\033[0m"
    echo "OBELISK_DOCKER_BASIC=true" >> $env_file
    export OBELISK_DOCKER_BASIC=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_BASIC=false!\033[0m"
    echo "OBELISK_DOCKER_BASIC=false" >> $env_file
    export OBELISK_DOCKER_BASIC=false
fi

if [ "$docker_cyclone_perf" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_CYCLONE_PERF=true!\033[0m"
    echo "OBELISK_DOCKER_CYCLONE_PERF=true" >> $env_file
    export OBELISK_DOCKER_CYCLONE_PERF=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_CYCLONE_PERF=false!\033[0m"
    echo "OBELISK_DOCKER_CYCLONE_PERF=false" >> $env_file
    export OBELISK_DOCKER_CYCLONE_PERF=false
fi

if [ "$docker_leap" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_LEAP=true!\033[0m"
    echo "OBELISK_DOCKER_LEAP=true" >> $env_file
    export OBELISK_DOCKER_LEAP=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_LEAP=false!\033[0m"
    echo "OBELISK_DOCKER_LEAP=false" >> $env_file
    export OBELISK_DOCKER_LEAP=false
fi

if [ "$docker_zed" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_ZED=true!\033[0m"
    echo "OBELISK_DOCKER_ZED=true" >> $env_file
    export OBELISK_DOCKER_ZED=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_ZED=false!\033[0m"
    echo "OBELISK_DOCKER_ZED=false" >> $env_file
    export OBELISK_DOCKER_ZED=false
fi

if [ "$docker_pixi" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_PIXI=true!\033[0m"
    echo "OBELISK_DOCKER_PIXI=true" >> $env_file
    export OBELISK_DOCKER_PIXI=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_PIXI=false!\033[0m"
    echo "OBELISK_DOCKER_PIXI=false" >> $env_file
    export OBELISK_DOCKER_PIXI=false
fi

if [ "$docker_unitree" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_UNITREE=true!\033[0m"
    echo "OBELISK_DOCKER_UNITREE=true" >> $env_file
    export OBELISK_DOCKER_UNITREE=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_UNITREE=false!\033[0m"
    echo "OBELISK_DOCKER_UNITREE=false" >> $env_file
    export OBELISK_DOCKER_UNITREE=false
fi

if [ "$docker_fr3" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_FR3=true!\033[0m"
    echo "OBELISK_DOCKER_FR3=true" >> $env_file
    export OBELISK_DOCKER_FR3=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_FR3=false!\033[0m"
    echo "OBELISK_DOCKER_FR3=false" >> $env_file
    export OBELISK_DOCKER_FR3=false
fi

if [ "$docker_group_leap" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_GROUP_LEAP=true!\033[0m"
    echo "OBELISK_DOCKER_GROUP_LEAP=true" >> $env_file
    export OBELISK_DOCKER_GROUP_LEAP=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_GROUP_LEAP=false!\033[0m"
    echo "OBELISK_DOCKER_GROUP_LEAP=false" >> $env_file
    export OBELISK_DOCKER_GROUP_LEAP=false
fi

if [ "$docker_group_zed" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_GROUP_ZED=true!\033[0m"
    echo "OBELISK_DOCKER_GROUP_ZED=true" >> $env_file
    export OBELISK_DOCKER_GROUP_ZED=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_GROUP_ZED=false!\033[0m"
    echo "OBELISK_DOCKER_GROUP_ZED=false" >> $env_file
    export OBELISK_DOCKER_GROUP_ZED=false
fi

if [ "$docker_group_fr3" = true ]; then
    echo -e "\033[1;32mSetting OBELISK_DOCKER_GROUP_FR3=true!\033[0m"
    echo "OBELISK_DOCKER_GROUP_FR3=true" >> $env_file
    export OBELISK_DOCKER_GROUP_FR3=true
else
    echo -e "\033[1;33mSetting OBELISK_DOCKER_GROUP_FR3=false!\033[0m"
    echo "OBELISK_DOCKER_GROUP_FR3=false" >> $env_file
    export OBELISK_DOCKER_GROUP_FR3=false
fi

# hardware credentials
if [ -n "$OBELISK_DOCKER_FR3_USERNAME" ]; then
    echo -e "\033[1;32mWriting OBELISK_DOCKER_FR3_USERNAME to .env\033[0m"
    echo "OBELISK_DOCKER_FR3_USERNAME=$OBELISK_DOCKER_FR3_USERNAME" >> $env_file
else
    echo -e "\033[1;33mOBELISK_DOCKER_FR3_USERNAME not set. Skipping.\033[0m"
fi

if [ -n "$OBELISK_DOCKER_FR3_PASSWORD" ]; then
    echo -e "\033[1;32mWriting OBELISK_DOCKER_FR3_PASSWORD to .env\033[0m"
    echo "OBELISK_DOCKER_FR3_PASSWORD=$OBELISK_DOCKER_FR3_PASSWORD" >> $env_file
else
    echo -e "\033[1;33mOBELISK_DOCKER_FR3_PASSWORD not set. Skipping.\033[0m"
fi

# copy scripts to the docker directory
cp $OBELISK_ROOT/scripts/install_sys_deps.sh $OBELISK_ROOT/docker/install_sys_deps.sh
cp $OBELISK_ROOT/scripts/config_groups.sh $OBELISK_ROOT/docker/config_groups.sh
cp $OBELISK_ROOT/scripts/user_setup.sh $OBELISK_ROOT/docker/user_setup.sh
