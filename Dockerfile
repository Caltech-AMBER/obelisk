# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV USERNAME=$USERNAME
ENV USER_UID=$USER_UID
ENV USER_GID=$USER_GID

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
    python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -d /home/${USERNAME} --shell /usr/bin/bash && \
    echo "${USERNAME}:password" | chpasswd && \
    usermod -aG sudo ${USERNAME} && \
    echo "%sudo ALL=NOPASSWD:/usr/bin/apt-get update, /usr/bin/apt-get upgrade, /usr/bin/apt-get install, /usr/bin/apt-get remove" >> /etc/sudoers

# switch to new user and workdir
USER ${USER_UID}

# pixi and nvm (for pyright)
RUN curl -fsSL https://pixi.sh/install.sh | bash && \
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash && \
    source /home/${USERNAME}/.bashrc && \
    export NVM_DIR="$HOME/.nvm" && \
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
    [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" && \
    nvm install 20

# add local user binary folder to PATH variable
ENV PATH="${PATH}:/home/${USERNAME}/.local/bin"
WORKDIR /home/${USERNAME}
