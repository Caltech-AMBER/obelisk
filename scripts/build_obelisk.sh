#!/bin/bash
if [ -z "$PIXI_ENVIRONMENT_NAME" ]; then
    echo -e "\033[1;31mYou must run this script from an activated pixi environment!\033[0m"
else
    pixi run messages-build
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash
    pixi run ros-build
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash
fi
