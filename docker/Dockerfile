# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USER=user
ARG UID=1000
ARG GID=1000
ENV USER=$USER
ENV UID=$UID
ENV GID=$GID

# set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# essential dependencies
RUN apt-get update -y && \
	apt-get install -y \
    curl \
    build-essential \
    cmake \
    clang-tools-12 \
    nano \
    vim \
    git \
    python3-dev \
    python-is-python3 \
    python3-pip \
    python3-argcomplete \
    sudo && \
    rm -rf /var/lib/apt/lists/*


# ../scripts/
COPY install-mj-source.sh /install-mj-source.sh
RUN chmod +x /install-mj-source.sh && /install-mj-source.sh --mujoco-dir /home/mujoco


# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# switch to new user and workdir
USER ${UID}

# pixi, uv, and nvm (for pyright)
RUN curl -fsSL https://pixi.sh/install.sh | bash && \
curl -LsSf https://astral.sh/uv/install.sh | sh && \
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash && \
    source /home/${USER}/.bashrc && \
    export NVM_DIR="$HOME/.nvm" && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" && \
    nvm install 20

# add local user binary folder to PATH variable
ENV PATH="${PATH}:/home/${USER}/.local/bin"
WORKDIR /home/${USER}
