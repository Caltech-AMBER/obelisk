#!/bin/bash

# only runs the following if the NOOB environment variable is set to true
if [ "$NOOB" != "true" ]; then
    echo -e "\033[1;33mNOOB=false, so we assume you don't want us to source obelisk for you!\033[0m"
    exit 0
fi

# source-rosbase = { cmd="bash source_rosbase.sh", cwd="scripts", env={ NOOB="true" } }
# ros-build = { cmd="colcon build --symlink-install --parallel-workers $(nproc)", cwd="obelisk_ws", inputs=["src"], depends-on=["source-rosbase"] }
# sources the obelisk installations if not already sourced
cmd="source $OBELISK_ROOT/obelisk_ws/install/setup.sh"
grep -qxF "$cmd" ~/.bashrc || (echo "$cmd" >> ~/.bashrc && $cmd && echo -e "\033[1;32mObelisk sourced!\033[0m")
