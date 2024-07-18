#!/bin/bash

source /opt/ros/humble/setup.bash

cd ${OBELISK_ROOT}/obelisk/python
pip install -e .
pip install "ruamel.yaml"

cd ${OBELISK_ROOT}/obelisk_ws
colcon build --symlink-install --packages-select obelisk_control_msgs obelisk_sensor_msgs obelisk_estimator_msgs obelisk_std_msgs
source install/setup.bash
colcon build --symlink-install --parallel-workers $(nproc)
source install/setup.bash
