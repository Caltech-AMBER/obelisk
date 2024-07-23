#!/bin/bash

# removes the build directory of the cpp obelisk project
rm -rf $OBELISK_ROOT/obelisk/cpp/build

# removes the build/install/log directories of the obelisk_ws
if [ -z "$PIXI_ENVIRONMENT_NAME" ]; then
    if [ -z "$OBELISK_ROOT" ]; then
        echo -e "\033[1;31mOBELISK_ROOT is not set. Run dev_setup.sh first!\033[0m"
        exit 0
    fi
    rm -rf $OBELISK_ROOT/obelisk_ws/build $OBELISK_ROOT/obelisk_ws/install $OBELISK_ROOT/obelisk_ws/log

else
    pixi run ros-clean
fi

echo -e "\033[1;32mDeleted the build/install/log dirs under obelisk_ws!\033[0m"
