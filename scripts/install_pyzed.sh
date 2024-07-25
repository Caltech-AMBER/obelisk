#!/bin/bash

# installs pyzed if it is not already installed
if python -c "import pyzed" &> /dev/null; then
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
fi
