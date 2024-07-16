# Visualization
Obelisk comes with a suite of tools for visualizing the robot. Some of these include:
- Robot visualization nodes to display robots in Rviz
- (todo) Custom rviz plugins for visualizing Obelisk messages

## Robot Visualization
Often times you may want to display a visual of a robot in a specified configuration, or see the motion of a robot throughout a trajectory, or maybe you want to check the estimated state vs the true state. In all of these cases we need to be able to see the robot. Obelisk provides two nodes to help with this:
- `ObeliskVizRobot` which is an abstract class, designed to be inherited
- `ObeliskVizRobotDefault` which is a concrete child of `ObeliskVizRobot` designed to work with most common estimated message types

These nodes are designed to be running when estimated state messages are being published. They can be configured to listen to any chosen topic. Once listening to this topic, they will parse incoming estimated state messages into a format that RViz understands. Concretely this means that the estimated state is being transformed into a `sensor_msgs/msg/JointState` message and a `tf2` transform between the fixed frame (default is `world`) and the base link on the robot. The `JointState` message is being published so that a `robot_state_publisher` (see [here](https://index.ros.org/p/robot_state_publisher/github-ros-robot_state_publisher/#humble)) can transform it into the transforms needed by Rviz.

With the proper configuration settings, the launch file will automatically bring up Rviz (with the desired configuration file), bring up the `robot_state_publisher` and the `ObeliskVizRobot`. Multiple robots can be brought up by specifying multiple `ObeliskVizRobots`.

In most cases, `ObeliskVizRobotDefault` should work fine. `ObeliskVizRobotDefault` requires an estimated state message with the following fields:
- `base_link_name`: name of the base link
- `q_base`: vector of (x, y, z, quat_x, quat_y, quat_z, quat_w). If length is zero then we assume the robot is fixed base
- `joint_names`: names of all the joints
- `q_joints`: vector of joint positions

The `base_link_name` must match a link in the URDF and `joint_names` must all be valid joint names in the URDF. If the estimated message does not have these fields, then you must extend `ObeliskVizRobot` and write the code to parse the message into the necessary transform and message.

### Visualization Configuration Settings
#### Example Configuration
```
  viz:
    on: True
    viz_tool: rviz
    rviz_pkg: obelisk_ros
    rviz_config: basic_obk_config.rviz
    viz_nodes:
      - pkg: obelisk_viz_cpp
        executable: default_robot_viz
        robot_pkg: g1_description
        urdf: g1.urdf
        robot_topic: robot_description
        tf_prefix: g1
        subscribers:
          - ros_parameter: sub_viz_est_setting
            topic: estimated_state
            history_depth: 10
            callback_group: None
            non_obelisk: False
        publishers:
          - ros_parameter: pub_viz_joint_setting
            topic: joint_states_g1
            history_depth: 10
            callback_group: None
        timers:
          - ros_parameter: timer_viz_joint_setting
            timer_period_sec: 0.05
            callback_group: None
      - pkg: obelisk_viz_cpp
        executable: default_robot_viz
        robot_pkg: go2_description
        urdf: go2_description.urdf
        robot_topic: go2_description
        tf_prefix: go2
        subscribers:
          - ros_parameter: sub_viz_est_setting
            topic: estimated_state2
            history_depth: 10
            callback_group: None
            non_obelisk: False
        publishers:
          - ros_parameter: pub_viz_joint_setting
            topic: joint_states_go2
            history_depth: 10
            callback_group: None
        timers:
          - ros_parameter: timer_viz_joint_setting
            timer_period_sec: 0.05
            callback_group: None
```
#### Breaking Down the Configuration
```
  viz:
    on: True
    rviz_pkg: obelisk_ros
    rviz_config: basic_obk_config.rviz
```
The visualization section starts with the `viz` tag. Then we have all the "global" visualization settings, i.e. all the settings that apply to everything, such as Rviz settings.
- `on` is a boolean flag to turn on or off the visualizer and spin the nodes. If this is false, all the following settings are skipped.
- `viz_tool` (optional) selects which visualization tool to bring up. For now the only two supported options are `rviz` and `foxglove`. If not present, the default is `rviz`.
- `rviz_pkg` (optional) is the package where the Rviz configuration file is found. Not need if `viz_tool` is `foxglove`, but required for `rviz`.
- `rviz_config` (optional) is the name of the Rviz configuration file. ***Note that we assume this is stored in a folder named `rviz` in the package listed above.*** Be sure that this folder is "installed" when building ROS. Not need if `viz_tool` is `foxglove`, but required for `rviz`.

Then under `viz_nodes` we have a list of nodes and their settings. We will examine only one as they always have the same fields.
```
- pkg: obelisk_viz_cpp
    executable: default_robot_viz
    robot_pkg: g1_description
    urdf: g1.urdf
    robot_topic: robot_description
    tf_prefix: g1
    subscribers:
        - ros_parameter: sub_viz_est_setting
        topic: estimated_state
        history_depth: 10
        callback_group: None
        non_obelisk: False
    publishers:
        - ros_parameter: pub_viz_joint_setting
        topic: joint_states_g1
        history_depth: 10
        callback_group: None
    timers:
        - ros_parameter: timer_viz_joint_setting
        timer_period_sec: 0.05
        callback_group: None
```
- `pkg` gives the package where the executable is.
- `executable` is the name of the executable.
- `robot_pkg` is the name of the package where all the robot files are stored.
- `urdf` is the name of the URDF file. ***Note that we assume the urdf is stored in a folder named `rviz` in the `robot_pkg`.***
- `robot_topic` (optional) is the name of the topic where the robot description (urdf) will be published for Rviz. This remaps the `robot_description` topic give in the [`robot_state_publisher`](https://index.ros.org/p/robot_state_publisher/github-ros-robot_state_publisher/#humble).
- `tf_prefix` (optional) is the prefix on all of the transform messages put out by the `robot_state_publisher`. In the `ObeliskVizRobotDefault` implementation this is accounted for. If you extend `ObeliskVizRobot` manually, then you must be sure to use this prefix (given as a node parameter) to prefix the base transform.
- Finally we have all the normal component settings, which we will not go over here.

The `robot_topic` and `tf_prefix` are mostly useful when you have multiple robots to display. By changing these options we can prevent name clashes.
