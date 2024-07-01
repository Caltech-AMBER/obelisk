#!/bin/bash

# only runs the following if the NOOB environment variable is set to true
if [ "$NOOB" != "true" ]; then
    echo -e "\033[1;33mNOOB=false, so we assume you don't want us to source obelisk for you!\033[0m"
    exit 0
fi

# sources the obelisk installations if not already sourced
cmd='
if which python | grep -q ".pixi"; then
    source $OBELISK_ROOT/obelisk_ws/install/setup.sh
fi
'
start_marker="# >>> source obelisk ROS2 setup.sh >>>"
end_marker="# <<< source obelisk ROS2 setup.sh <<<"
if grep -Fq "$start_marker" ~/.bashrc && grep -Fq "$end_marker" ~/.bashrc; then
    awk -v start="$start_marker" -v end="$end_marker" -v cmd="$cmd" '
        $0 == start {print; print cmd; f=1; next}
        $0 == end {f=0}
        !f
    ' ~/.bashrc > ~/.bashrc.tmp && mv ~/.bashrc.tmp ~/.bashrc
else
    printf '\n%s\n%s\n%s\n' "$start_marker" "$cmd" "$end_marker" >> ~/.bashrc
fi
source ~/.bashrc
echo -e "\033[1;32mObelisk sourced!\033[0m"
