#!/bin/bash

if [ -z "$PIXI_ENVIRONMENT_NAME" ]; then
    if [ -z "$OBELISK_ROOT" ]; then
        echo -e "/033[1;31mOBELISK_ROOT is not set. Run dev_setup.sh first!\033[0m"
        exit 0
    fi
    
    echo -e "\033[1;32mBuilding Obelisk messages outside of a pixi env...\033[0m"
    cd $OBELISK_ROOT/obelisk_ws
    colcon build --symlink-install --parallel-workers $(nproc) \
        --packages-select obelisk_control_msgs obelisk_estimator_msgs obelisk_sensor_msgs obelisk_std_msgs
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash
    colcon build --symlink-install --parallel-workers $(nproc) \
        --packages-skip obelisk_control_msgs obelisk_estimator_msgs obelisk_sensor_msgs obelisk_std_msgs
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash

else
    echo -e "\033[1;32mBuilding Obelisk messages within a pixi env...\033[0m"
    pixi run messages-build
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash
    pixi run ros-build
    source $OBELISK_ROOT/obelisk_ws/install/setup.bash
fi

echo -e "\033[1;32mObelisk built successfully!\033[0m"
