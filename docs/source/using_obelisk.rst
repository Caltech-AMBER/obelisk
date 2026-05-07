=============================
Using Obelisk - Basic Example
=============================

Obelisk supplies libraries in both C++ and Python which can be easily installed on your system and used as normal libraries. Obelisk also provides a default ROS2 workspace that can be an `underlay <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay>`_ or included as a package dependency directly in your ROS2 workspace. The Obelisk workspace will provide access to all the messages and launch files whereas the libraries will allow you to write custom `ObeliskNodes`. For concrete examples, see `this repository <https://github.com/Caltech-AMBER/obelisk-examples>`_.

The libraries provide five main class interfaces:

- ``ObeliskController``
- ``ObeliskEstimator``
- ``ObeliskRobot``
- ``ObeliskSensor``
- ``ObeliskNode``

In your code, you can write classes that inherit from these classes and thus gain the benefit of being an ``ObeliskNode``.

Below, we will walk through an example where we write code to control a simple two-link robot with a single actuator.

Controller Code
^^^^^^^^^^^^^^^
.. tabs::
    .. tab:: C++
        .. code-block:: C++

            #include "rclcpp/rclcpp.hpp"

            #include "obelisk_controller.h"
            #include "obelisk_ros_utils.h"

            class PositionSetpointController
              : public obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                  obelisk_estimator_msgs::msg::EstimatedState> {
              public:
              PositionSetpointController(const std::string& name)
                  : obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                              obelisk_estimator_msgs::msg::EstimatedState>(name) {}

              protected:
              void UpdateXHat(__attribute__((unused)) const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}

              obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
                  obelisk_control_msgs::msg::PositionSetpoint msg;

                  msg.u.clear();
                  rclcpp::Time time = this->get_clock()->now();
                  double time_sec   = time.seconds();

                  msg.u.emplace_back(sin(time_sec));

                  this->GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>(this->ctrl_key_)->publish(msg);

                  return msg;
              };
            };

        Here we can see that ``PositionSetpointController`` inherits from ``ObeliskController`` and implements the two abstract methods required. ``ObeliskController`` is templated on the control message type and the state estimator message type.

        ``ComputeControl`` and ``UpdateXHat`` are automatically registered as callbacks for the timer and subscriber respectively.

    .. tab:: Python
        .. code-block:: python

            import numpy as np
            from obelisk_control_msgs.msg import PositionSetpoint
            from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

            from obelisk_py.core.control import ObeliskController
            from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound


            class ExamplePositionSetpointController(ObeliskController):
                """Example position setpoint controller."""

                def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
                    """Initialize the example position setpoint controller."""
                    super().__init__(node_name)

                def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
                    """Configure the controller."""
                    super().on_configure(state)
                    self.x_hat = None
                    return TransitionCallbackReturn.SUCCESS

                def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
                    """Update the state estimate.

                    Parameters:
                        x_hat_msg: The Obelisk message containing the state estimate.
                    """
                    pass  # do nothing

                def compute_control(self) -> ObeliskControlMsg:
                    """Compute the control signal for the dummy 2-link robot.

                    Returns:
                        obelisk_control_msg: The control message.
                    """
                    # computing the control input
                    u = np.sin(self.t)  # example state-independent control input

                    # setting the message
                    position_setpoint_msg = PositionSetpoint()
                    position_setpoint_msg.u = [u]
                    self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
                    assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
                    return position_setpoint_msg  # type: ignore

Estimator Code
^^^^^^^^^^^^^^

.. tabs::
    .. tab:: C++
        .. code-block:: C++

          #include "rclcpp/rclcpp.hpp"

          #include "obelisk_estimator.h"
          #include "obelisk_ros_utils.h"

          class JointEncodersPassthroughEstimator
              : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
            public:
            JointEncodersPassthroughEstimator(const std::string& name)
                : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {

                this->RegisterObkSubscription<obelisk_sensor_msgs::msg::JointEncoders>(
                    "sub_sensor",
                    std::bind(&JointEncodersPassthroughEstimator::JointEncoderCallback, this, std::placeholders::_1));
            }

            protected:
            void JointEncoderCallback(const obelisk_sensor_msgs::msg::JointEncoders& msg) { joint_encoders_ = msg.y; }

            obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
                obelisk_estimator_msgs::msg::EstimatedState msg;

                msg.x_hat = joint_encoders_;

                this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);

                return msg;
            };

            private:
            std::vector<double> joint_encoders_;
          };

        Here we can see that ``JointEncodersPassthroughEstimator`` inherits from ``ObeliskEstimator`` and implements the one abstract method required. ``ObeliskEstimator`` is templated on the estimated message type.

        ``ComputeStateEstimate`` is automatically registered as callbacks for timer. ``JointEncoderCallback`` is not automatically registered as a callback since it it not a part of a required component in the node. We can see that it is registered with the ``std::bind()`` call in the constructor.
    .. tab:: Python
        .. code-block:: python

            from typing import Union

            from obelisk_estimator_msgs.msg import EstimatedState
            from obelisk_sensor_msgs.msg import JointEncoders
            from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

            from obelisk_py.core.estimation import ObeliskEstimator


            class JointEncodersPassthroughEstimator(ObeliskEstimator):
                """Passthrough estimator for joint encoder sensors."""

                def __init__(self, node_name: str = "joint_encoders_passthrough_estimator") -> None:
                    """Initialize the joint encoders passthrough estimator."""
                    super().__init__(node_name)
                    self.register_obk_subscription(
                        key="sub_sensor",
                        callback=self.joint_encoder_callback,  # type: ignore
                        msg_type=JointEncoders,
                    )

                def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
                    """Configure the estimator."""
                    super().on_configure(state)
                    self.joint_encoder_values = None
                    return TransitionCallbackReturn.SUCCESS

                def joint_encoder_callback(self, msg: JointEncoders) -> None:
                    """Callback for joint encoder messages."""
                    self.joint_encoder_values = msg.y

                def compute_state_estimate(self) -> Union[EstimatedState, None]:
                    """Compute the state estimate."""
                    estimated_state_msg = EstimatedState()
                    if self.joint_encoder_values is not None:
                        estimated_state_msg.x_hat = self.joint_encoder_values
                        self.obk_publishers["publisher_est"].publish(estimated_state_msg)
                        return estimated_state_msg

Spinning all the Nodes
^^^^^^^^^^^^^^^^^^^^^^
Obelisk comes with a helper function ``SpinObelisk`` to make spinning up your nodes easy.

.. tabs::
    .. tab:: C++
        Controller spin up:

        .. code-block:: C++

            #include "obelisk_ros_utils.h"
            #include "position_setpoint_controller.h"

            int main(int argc, char* argv[]) {
                obelisk::utils::SpinObelisk<PositionSetpointController, rclcpp::executors::MultiThreadedExecutor>(
                    argc, argv, "position_setpoint_controller");
            }

        State estimator spin up:

        .. code-block:: C++

            #include "jointencoders_passthrough_estimator.h"
            #include "obelisk_ros_utils.h"

            int main(int argc, char* argv[]) {
                obelisk::utils::SpinObelisk<JointEncodersPassthroughEstimator, rclcpp::executors::MultiThreadedExecutor>(
                    argc, argv, "passthrough_estimator");
            }

        Simulation spin up:

        .. code-block:: C++

            #include "rclcpp/rclcpp.hpp"

            #include "obelisk_mujoco_sim_robot.h"
            #include "obelisk_ros_utils.h"

            int main(int argc, char* argv[]) {
                obelisk::utils::SpinObelisk<obelisk::ObeliskMujocoRobot<obelisk_control_msgs::msg::PositionSetpoint>,
                                            rclcpp::executors::MultiThreadedExecutor>(argc, argv, "mujoco_sim");
            }

    .. tab:: Python
        Controller spin up:

        .. code-block:: python

            from typing import List, Optional

            from rclpy.executors import SingleThreadedExecutor

            from obelisk_py.core.utils.ros import spin_obelisk
            from obelisk_py.zoo.control.example.example_position_setpoint_controller import ExamplePositionSetpointController


            def main(args: Optional[List] = None) -> None:
                """Main entrypoint."""
                spin_obelisk(args, ExamplePositionSetpointController, SingleThreadedExecutor)


            if __name__ == "__main__":
                main()

        State estimator spin up:

        .. code-block:: python

            from typing import List, Optional

            from rclpy.executors import SingleThreadedExecutor

            from obelisk_py.core.utils.ros import spin_obelisk
            from obelisk_py.zoo.estimation.jointencoders_passthrough_estimator import JointEncodersPassthroughEstimator


            def main(args: Optional[List] = None) -> None:
                """Main entrypoint."""
                spin_obelisk(args, JointEncodersPassthroughEstimator, SingleThreadedExecutor)


            if __name__ == "__main__":
                main()

        Simulation spin up:

        .. code-block:: python

            from typing import List, Optional

            from rclpy.executors import MultiThreadedExecutor

            from obelisk_py.core.sim.mujoco import ObeliskMujocoRobot
            from obelisk_py.core.utils.ros import spin_obelisk


            def main(args: Optional[List] = None) -> None:
                """Main entrypoint."""
                spin_obelisk(args, ObeliskMujocoRobot, MultiThreadedExecutor)


            if __name__ == "__main__":
                main()

Obelisk Configuration File
^^^^^^^^^^^^^^^^^^^^^^^^^^
Obelisk nodes can be easily configured via a Obelisk configuration (yaml) file. An example Obelisk configuration file is given here.

.. code-block:: yaml

  config: dummy
  onboard:
    control:
      - pkg: obelisk_control_cpp
        executable: example_position_setpoint_controller
        params_path: /obelisk_ws/src/obelisk_ros/config/dummy_params.txt
        # callback_groups:
        publishers:
          - key: pub_ctrl
            topic: /obelisk/dummy/ctrl
            history_depth: 10
            callback_group: None
        subscribers:
          - key: sub_est
            topic: /obelisk/dummy/est
            history_depth: 10
            callback_group: None
        timers:
          - key: timer_ctrl
            timer_period_sec: 0.001
            callback_group: None
    estimation:
      - pkg: obelisk_estimation_cpp
        executable: jointencoders_passthrough_estimator
        # callback_groups:
        publishers:
          - key: pub_est
            topic: /obelisk/dummy/est
            history_depth: 10
            callback_group: None
        subscribers:
          - key: sub_sensor
            topic: /obelisk/dummy/sensor
            history_depth: 10
            callback_group: None
        timers:
          - key: timer_est
            timer_period_sec: 0.001
            callback_group: None
    # sensing:
    robot:
      - is_simulated: True
        pkg: obelisk_sim_cpp
        executable: obelisk_mujoco_robot
        params:
          ic_keyframe: ic
        # callback_groups:
        # publishers:
        subscribers:
          - key: sub_ctrl
            topic: /obelisk/dummy/ctrl
            history_depth: 10
            callback_group: None
        sim:
          - ros_parameter: mujoco_setting
            model_xml_path: dummy/dummy.xml
            num_steps_per_viz: 5
            sensor_settings:
            - topic: /obelisk/dummy/joint_encoders
              dt: 0.001
              msg_type: ObkJointEncoders
              sensor_names:
                joint_pos: jointpos
                joint_vel: jointvel
            - topic: /obelisk/dummy/imu
              dt: 0.002
              msg_type: Imu
              sensor_names:
                tip_acc_sensor: accelerometer
                tip_gyro_sensor: gyro
                tip_frame_sensor: framequat
            - topic: /obelisk/dummy/framepose
              dt: 0.002
              msg_type: ObkFramePose
              sensor_names:
                tip_pos_sensor: framepos
                tip_orientation_sensor: framequat
            viz_geoms:
              dt: 1.0
              dummy_box: box
              dummy_box_2: box
              dummy_sphere: sphere



Breaking down the configuration file
------------------------------------
.. code-block:: yaml

  config: dummy
  onboard:


First we give the name of this configuration (``dummy``), and which device this is running on.

.. code-block:: yaml

  control:
    pkg: obelisk_control_py
    executable: example_position_setpoint_controller
    params_path: /obelisk_ws/src/obelisk_ros/config/dummy_params.txt
    publishers:
      - key: pub_ctrl
        topic: /obelisk/dummy/ctrl
        history_depth: 10
        callback_group: None
    subscribers:
      - key: sub_est
        topic: /obelisk/dummy/est
        history_depth: 10
        callback_group: None
    timers:
      - key: timer_ctrl
        timer_period_sec: 0.001
        callback_group: None


Now we configure our Controller node. ``pkg`` gives the name of the package containing the Obelisk node, and ``executable`` tells us what the name is of the executable with ``main`` in it.
``params_path`` (optional) is a string parameter that allows you to specify a file path that can be accessed within your code. This is useful for things like accessing controller gains that are specfied through a seperate yaml file. Note that there is no convention on how the file path is processed as that is up to you as the user.

The launch-side producer bundles the ``publishers``, ``subscribers``, ``timers``, and ``callback_groups`` sections of each node into a single ROS string parameter named ``obelisk_settings`` whose value is a YAML document. Each Obelisk node parses that YAML at ``on_configure`` time and looks up its registered components by ``key``. (For backward compatibility with older configs, the producer still accepts ``ros_parameter: <name>_setting`` and derives ``key`` by stripping the ``_setting`` suffix.)

Publishers and subscribers have the following options:

- ``key`` gives the string key associated with the component. It must match the key the node registered via ``register_obk_publisher`` / ``register_obk_subscription`` (Python) or ``RegisterObkPublisher`` / ``RegisterObkSubscription`` (C++).
- ``topic`` gives the string topic name that will either be published or subscribed to.
- ``history_depth`` (optional) gives the number of messages to hold in the queue before deleting additional messages. If this is not set we use the default value of 10.
- ``callback_group`` (optional) gives the string name of the callback group to use. The callback groups can be configured within this configuration file. If no value is specified, then the node's default callback group is used.

Note: the message type is no longer specified in YAML for normal pub/sub entries — it is bound at code-registration time (the ``msg_type`` argument to ``register_obk_publisher`` in Python, or the template parameter in C++).

Timers have the following options:

- ``key`` gives the string key associated with the timer (matches the key used in ``register_obk_timer`` / ``RegisterObkTimer``).
- ``timer_period_sec`` gives the period of the timer in seconds.
- ``callback_group`` (optional) gives the string name of the callback group to use. The callback groups can be configured within this configuration file. If no value is specified, then the node's default callback group is used.

This is repeated for every non-system node in the block diagram, which in this case is just an additional estimator.

.. code-block:: yaml

  estimation:
      pkg: obelisk_estimation_py
      executable: jointencoders_passthrough_estimator
      publishers:
        - key: pub_est
          topic: /obelisk/dummy/est
          history_depth: 10
          callback_group: None
      subscribers:
        - key: sub_sensor
          topic: /obelisk/dummy/sensor
          history_depth: 10
          callback_group: None
      timers:
        - key: timer_est
          timer_period_sec: 0.001
          callback_group: None


Lastly, we need to configure the ``robot`` (aka, the system).

.. code-block:: yaml

  robot:
    is_simulated: True
    pkg: obelisk_sim_py
    executable: obelisk_mujoco_robot
    params:
      ic_keyframe: ic
    # callback_groups:
    # publishers:
    subscribers:
      - key: sub_ctrl
        topic: /obelisk/dummy/ctrl
        history_depth: 10
        callback_group: None
    sim:
      - ros_parameter: mujoco_setting
        model_xml_path: dummy/dummy.xml
        num_steps_per_viz: 5
        sensor_settings:
        - topic: /obelisk/dummy/joint_encoders
          dt: 0.001
          msg_type: ObkJointEncoders
          sensor_names:
            joint_pos: jointpos
            joint_vel: jointvel
        - topic: /obelisk/dummy/imu
          dt: 0.002
          msg_type: Imu
          sensor_names:
            tip_acc_sensor: accelerometer
            tip_gyro_sensor: gyro
            tip_frame_sensor: framequat
        - topic: /obelisk/dummy/framepose
          dt: 0.002
          msg_type: ObkFramePose
          sensor_names:
            tip_pos_sensor: framepos
            tip_orientation_sensor: framequat
        viz_geoms:
          dt: 1.0
          dummy_box: box
          dummy_box_2: box
          dummy_sphere: sphere

``is_simulated`` marks if we are running on hardware or in simulation. ``pkg`` and ``executable`` are as before.
``ic_keyframe`` (optional) in the params section tells the simulation which keyframe to use for an initial condition.

Now, we must configure the Components of the node, which in this example is just a subscriber. These Components have all the same options as the non-system Components given above.

Lastly, since this is a simulation, we must provide the simulator with all relevant information. Here, we are using the Mujoco simulation interface. The simulator's settings are passed as a YAML-formatted string in their own ROS parameter (``mujoco_setting``), which is why the ``sim:`` entry still uses ``ros_parameter:``. The new settings here are:

- ``num_steps_per_viz`` (optional) gives the number of steps to use between simulation rendering. If no value is provided, the default value of 8 steps will be used.

``sensor_settings`` is how we can specify what sensors our robot has. Within ``sensor_settings`` we have the following new options:

- ``msg_type`` gives the ROS message type associated with the given group of sensors.
- ``dt`` gives the sensor publishing period in seconds.
- Each element under ``sensor_names`` follows ``sensor_name: sensor_type`` **Note that the Mujoco XML must have all the sensors listed in the Obelisk configuration file, if you request a sensor here that is not available in Mujoco, there will be an error.** All supported Mujoco sensors and corresponding message types are listed below.

=============================================== ============================
Mujoco sensor type                              Message type (``msg_type``)
=============================================== ============================
``jointpos`` and ``jointvel``                   ``ObkJointEncoders``
``accelerometer``, ``gyro``, and ``framequat``  ``Imu`` (``sensor_msgs/Imu``)
``framepos`` and ``framequat``                  ``ObkFramePose``
=============================================== ============================

You may have multiple of the same type of sensor in the yaml.

- ``viz_geoms`` (optional) gives a list of visualization geometries that you want the simulation node to publish. The node will read the state of these geoms from the simulator and publish them so an external visualizer can see them. This is designed mostly for visualizing the environment, not the robot.

Thats it! Now we have configured our Obelisk nodes.
