#!/bin/bash

# if /usr/local/zed/get_python_api.py does not exist, local install was not run
# therefore, exit and don't attempt to install
if [ ! -f /usr/local/zed/get_python_api.py ]; then
    echo -e "\033[1;33mZED SDK not installed! Run the local install script first.\033[0m"
    exit 0
fi

# installs pyzed if it is not already installed
if python -c "import pyzed" &> /dev/null; then
    echo -e "\033[1;33mPyZED confirmed to be installed!\033[0m"
    exit 0
else
    curr_dir=$(pwd)
    pip install requests
    mkdir -p $OBELISK_ROOT/tmp
    cp /usr/local/zed/get_python_api.py $OBELISK_ROOT/tmp/get_python_api.py
    cd $OBELISK_ROOT/tmp
    python $OBELISK_ROOT/tmp/get_python_api.py
    rm -r $OBELISK_ROOT/tmp
    cd $curr_dir

    echo -e "\033[1;32mPyZED installed successfully!\033[0m"
fi
