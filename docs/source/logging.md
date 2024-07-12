# Logging

Obelisk uses the ROS2 logging system. Because of this, we highly encourage all users to also use the ROS2 logging system when writing their own nodes. Specifics on how this can be done can be found [here](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html). Obelisk also provides automatic logging directory creation and organization.

Whenever the launch file is used (i.e. `obk-launch`) a few steps occur:
1. Obelisk checks for a directory called `obk_logs` in `$ROS_HOME`. If `$ROS_HOME` is unset, then we use `$OBELISK_ROOT`. If the `obk_logs` directory is not present, then Obelisk will create this directory.
2. Obelisk will create a sub-folder in `obk_logs` for each run. The naming convention is `<config name>_<YYYYMMDD>_<HHMMSS>`. This is where all the log files will go (as a by-product, the `$ROS_LOG_DIR` environment variable is set to this folder).
3. If `bag` is set to `true` then Obelisk will automatically bag all of the topics and place them in a sub-folder called `obk_stack_bag`. ***Note that the `bag` option defaults to `true`, resulting in an opt-out system for bagging.***

Given the above logic, the default location for `obk_logs` will be in `$OBELISK_ROOT` and if you prefer to have the logs in your workspace, you need to set `$ROS_HOME`. Due to this, it is suggested to add `obk_logs/` to your `.gitignore` if you set `$ROS_HOME`.

Logs to the terminal and to this folder are generated every run.

The default log level used is `INFO`. If you would like to adjust the logging severity, you can do so in the source code for your node. For example in C++ to set to `DEBUG`:
```
auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
}
```
For the full example, see [here](https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_usage_component.cpp).
