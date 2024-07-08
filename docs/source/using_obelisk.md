# Using Obelisk
Obelisk supplies libraries in both C++ and Python which can be easily installed on your system and used as normal libraries. Obelisk also provides a default ROS2 workspace that can be an [underlay](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay) or included as a package dependency directly in your ROS2 workspace. The Obelisk workspace will provide access to all the messages and launch files whereas the libraries will allow you to write custom `ObeliskNodes`.

The libraries provide five main class interfaces:
- `ObeliskController`
- `ObeliskEstimator`
- `ObeliskRobot`
- `ObeliskSensor`
- `ObeliskNode`

In your code, you can write classes that inherit from these classes and thus gain the benefit of being an `ObeliskNode`. Specific examples and code-level details are given at TBD.

## Obelisk Configuration File
Obelisk nodes can be easily configured via a Obelisk configuration (yaml) file. An example Obelisk configuration file is given here.

```
config: dummy
onboard:
  control:
    impl: python
    executable: example_position_setpoint_controller
    publishers:
      - ros_parameter: pub_ctrl_setting
        topic: /obelisk/dummy/ctrl
        msg_type: PositionSetpoint
        key: "asdf"
        history_depth: 10
        callback_group: None
        non_obelisk: False
    subscribers:
      - ros_parameter: sub_est_setting
        topic: /obelisk/dummy/est
        msg_type: EstimatedState
        history_depth: 10
        callback_group: None
        non_obelisk: False
    timers:
      - ros_parameter: timer_ctrl_setting
        timer_period_sec: 0.001
        callback_group: None
  estimation:
    impl: python
    executable: jointencoders_passthrough_estimator
    publishers:
      - ros_parameter: pub_est_setting
        topic: /obelisk/dummy/est
        msg_type: EstimatedState
        history_depth: 10
        callback_group: None
        non_obelisk: False
    subscribers:
      - ros_parameter: sub_sensor_setting
        # key: sub1
        topic: /obelisk/dummy/sensor
        msg_type: JointEncoders
        history_depth: 10
        callback_group: None
        non_obelisk: False
    timers:
      - ros_parameter: timer_est_setting
        timer_period_sec: 0.001
        callback_group: None
  # sensing:
  robot:
    is_simulated: True
    impl: python
    executable: obelisk_mujoco_robot
    # callback_groups:
    # publishers:
    subscribers:
      - ros_parameter: sub_ctrl_setting
        # key: sub1
        topic: /obelisk/dummy/ctrl
        msg_type: PositionSetpoint
        history_depth: 10
        callback_group: None
        non_obelisk: False
    sim:
      - ros_parameter: mujoco_setting
        model_xml_path: dummy/dummy.xml
        n_u: 1
        time_step: 0.002
        num_steps_per_viz: 5
        sensor_settings:
        - topic: /obelisk/dummy/sensor
          dt: 0.001
          sensor_type: jointpos
          sensor_names:
          - sensor_joint1
```

### Breaking down the configuration file
```
config: dummy
onboard:
```

First we give the name of this configuration (`dummy`), and which device this is running on.

```
control:
    impl: python
    executable: example_position_setpoint_controller
    publishers:
      - ros_parameter: pub_ctrl_setting
        topic: /obelisk/dummy/ctrl
        msg_type: PositionSetpoint
        key: "asdf"
        history_depth: 10
        callback_group: None
        non_obelisk: False
    subscribers:
      - ros_parameter: sub_est_setting
        topic: /obelisk/dummy/est
        msg_type: EstimatedState
        history_depth: 10
        callback_group: None
        non_obelisk: False
    timers:
      - ros_parameter: timer_ctrl_setting
        timer_period_sec: 0.001
        callback_group: None
```

Now we configure our Controller node. `impl` gives the implementation language, and `executable` tells us what the name is of the executable with `main` in it. Now we need to configure all of the Components in this node. Publishers and subscribers have the following options.
- `ros_parameter` gives the string name of the ros parameter declared in the code. This is how the launch file get these options to the correct node.
- `topic` gives the string topic name that will either be published or subscribed to.
- `msg_type` gives the type of message we want to publish or subscribe to. ***Note this is only ever used in the Python implementation. In C++ the message type *must* be specified in the code as a templated parameter.***
- `key` TBD
- `history_depth` (optional) gives the number of messages to hold in teh queue before deleting additional messages. If this not set we the use the default value of 10.
- `callback_group` (optional) gives the string name of the callback group to use. The callback groups can be configured within this configuration file. If no value is specified, then the node's default callback group is used.
- `non_obelisk` (optional) determine whether this node can publish non-obelisk messages. ***Note if this is set to true, then this may cause problems with the interfaces, so only use if you are sure this is what you need.*** If no value is specified, then the default value of `False` is used.

Timers have the following options.
- `ros_parameter` gives the string name of the ros parameter declared in the code. This is how the launch file get these options to the correct node.
- `timer_period_sec` gives the period of the timer in seconds
- `callback_group` (optional) gives the string name of the callback group to use. The callback groups can be configured within this configuration file. If no value is specified, then the node's default callback group is used.

This is repeated for every non-system node in the block diagram, which in this case is just an additional estimator.

```
estimation:
    impl: python
    executable: jointencoders_passthrough_estimator
    publishers:
      - ros_parameter: pub_est_setting
        topic: /obelisk/dummy/est
        msg_type: EstimatedState
        history_depth: 10
        callback_group: None
        non_obelisk: False
    subscribers:
      - ros_parameter: sub_sensor_setting
        # key: sub1
        topic: /obelisk/dummy/sensor
        msg_type: JointEncoders
        history_depth: 10
        callback_group: None
        non_obelisk: False
    timers:
      - ros_parameter: timer_est_setting
        timer_period_sec: 0.001
        callback_group: None
```

The finally we need to configure the `robot` (aka the system).

```
  robot:
    is_simulated: True
    impl: python
    executable: obelisk_mujoco_robot
    # callback_groups:
    # publishers:
    subscribers:
      - ros_parameter: sub_ctrl_setting
        # key: sub1
        topic: /obelisk/dummy/ctrl
        msg_type: PositionSetpoint
        history_depth: 10
        callback_group: None
        non_obelisk: False
    sim:
      - ros_parameter: mujoco_setting
        model_xml_path: dummy/dummy.xml
        n_u: 1
        time_step: 0.002
        num_steps_per_viz: 5
        sensor_settings:
        - topic: /obelisk/dummy/sensor
          dt: 0.001
          sensor_type: jointpos
          sensor_names:
          - sensor_joint1
```

`is_simulated` marks if we are running on hardware or in simulation. `impl` again tells us which language the interface is written in. `executable` once again gives the name of the executable with `main` in it.

Now we must configure the Components of the node, which in this example is just a subscriber. These Components have all the same options as the non-system Components given above.

Lastly, since this is a simulation we must provide the simulator with all relevant information. Here we are using the Mujoco simulation interface. The new settings here are
- `n_u` gives the number of control inputs (i.e. the number of scalars)
- `time_step` (optional) gives the length of a simulation time step. If no value is provided, the default value of 0.002 seconds will be used.
- `num_steps_per_viz` (optional) gives the number of steps to use between simulation rendering. If no value is provided, the default value of 8 steps will be used.

`sensor_settings` is how we can specify what sensors our robot has. Within `sensor_settings` we have the following new options:
- `dt` gives the sensor publishing period in seconds.
- `sensor_type` gives the Mujoco sensor type. Each sensor type corresponds to a specific Obelisk message type. ***Note that the Mujoco XML must have all the sensors listed in the Obelisk configuration file, if you request a sensor here that is not available in Mujoco, there will be an error.*** All supported Mujoco sensors and corresponding Obelisk messages are listed below.

| Mujoco sensor type | Obelisk Message Type |
| ------------------ | -------------------- |
| `jointpos`         | `JointEncoders`      |

- `sensor_names` gives the names of the sensors to query, as given in the Mujoco XML.

Thats it! Now we have configured our Obelisk nodes.
