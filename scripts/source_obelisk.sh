#!/bin/bash

# only runs the following if the NOOB environment variable is set to true
if [ "$NOOB" != "true" ]; then
    echo -e "\033[1;33mNOOB=false, so we assume you don't want us to source obelisk for you!\033[0m"
    exit 0
fi

# sources the obelisk installations if not already sourced
cmd="source $(pwd)/install/setup.sh"
grep -qxF "$cmd" ~/.bashrc || echo "$cmd" >> ~/.bashrc && echo -e "\033[1;32mObelisk sourced!\033[0m"
