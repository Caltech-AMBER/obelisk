#!/bin/bash

# only runs the following if the NOOB environment variable is set to true
if [ "$NOOB" != "true" ]; then
    echo -e "\033[1;33mNOOB=false, so we assume you don't want us to source obelisk for you!\033[0m"
    exit 0
fi

if [ "$GLOBAL" != "true" ]; then
    echo -e "\033[1;33mGLOBAL=false, so Obelisk is only sourced in pixi envs! \
Avoids issue where the pixi python path is prepended to PATH globally.\033[0m"
else
    echo -e "\033[1;33mGLOBAL=true, so Obelisk is sourced globally! \
This means that the pixi python path is prepended to PATH unconditionally! \
This will affect your conda environments, so be careful!\033[0m"
fi


# sources the obelisk installations if not already sourced
cmd_local='if which python | grep -q ".pixi"; then
    source $OBELISK_ROOT/obelisk_ws/install/setup.sh
fi'
cmd_global='source $OBELISK_ROOT/obelisk_ws/install/setup.sh'
start_marker="# >>> source obelisk ROS2 setup.sh >>>"
end_marker="# <<< source obelisk ROS2 setup.sh <<<"
if [ "$GLOBAL" == "true" ]; then
    cmd="$cmd_global"
else
    cmd="$cmd_local"
fi
if grep -Fq "$start_marker" ~/.bashrc && grep -Fq "$end_marker" ~/.bashrc; then
    awk -v start="$start_marker" -v end="$end_marker" -v cmd="$cmd" '
        $0 == start {print; print cmd; f=1; next}
        $0 == end {f=0}
        !f
    ' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
else
    printf '\n%s\n%s\n%s\n' "$start_marker" "$cmd" "$end_marker" >> ~/.bashrc
fi

# makes cyclone DDS the default ROS2 middleware
cmd_local='if which python | grep -q ".pixi"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi'
cmd_global='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
start_marker="# >>> make cyclone DDS the rmw default >>>"
end_marker="# <<< make cyclone DDS the rmw default <<<"
if [ "$GLOBAL" == "true" ]; then
    cmd="$cmd_global"
else
    cmd="$cmd_local"
fi
if grep -Fq "$start_marker" ~/.bashrc && grep -Fq "$end_marker" ~/.bashrc; then
    awk -v start="$start_marker" -v end="$end_marker" -v cmd="$cmd" '
        $0 == start {print; print cmd; f=1; next}
        $0 == end {f=0}
        !f
    ' ~/.bashrc > ~/.bashrc.tmp && cp ~/.bashrc.tmp ~/.bashrc && rm ~/.bashrc.tmp
else
    printf '\n%s\n%s\n%s\n' "$start_marker" "$cmd" "$end_marker" >> ~/.bashrc
fi

source ~/.bashrc
echo -e "\033[1;32mObelisk sourced!\033[0m"
