from abc import ABC, abstractmethod

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.node import ObeliskNode
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg


class ObeliskController(ABC, ObeliskNode):
    """Abstract Obelisk controller node.

    Obelisk controllers are stateful. That is, all the quantities required to compute the control signal are stored in
    the controller object itself. This is done because the processes generating each of these quantities may act
    asynchronously. Similarly, the control action may be queried asynchronously.

    When implementing a new ObeliskController, the user should declare all quantities required to compute the control
    input in the constructor. These quantities should be updated by various update_X methods. Finally, the
    compute_control method should be implemented to compute the control signal using the updated quantities. Note that
    the control message should be of type ObeliskControlMsg to be compatible with the Obelisk ecosystem.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk controller.

        Parameters:
            node_name: The name of the node.

        Required ROS Parameters:
            msg_type_ctrl: The type of the control message. Passed as a string, then converted to the appropriate type.
            msg_type_est: The type of the state estimate message. Passed as string as above.

        Optional ROS Parameters:
            history_depth_ctrl: The depth of the control message history.
            history_depth_est: The depth of the state estimate message history.
            cb_group_ctrl: The callback group for the control message publisher and timer. Passed as string.
            cb_group_est: The callback group for the state estimate message subscriber.
        """
        super().__init__(node_name)

        # declare config string parameters
        self.declare_parameter("timer_ctrl_config_str", rclpy.Parameter.Type.STRING)
        self.declare_parameter("pub_ctrl_config_str", rclpy.Parameter.Type.STRING)
        self.declare_parameter("sub_est_config_str", rclpy.Parameter.Type.STRING)

        # stateful quantities
        self.x_hat = None  # the most recent state estimate

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)

        # parsing config strings
        self.timer_ctrl_config_str = self.get_parameter("timer_ctrl_config_str").get_parameter_value().string_value
        self.pub_ctrl_config_str = self.get_parameter("pub_ctrl_config_str").get_parameter_value().string_value
        self.sub_est_config_str = self.get_parameter("sub_est_config_str").get_parameter_value().string_value

        # create publishers+timers/subscribers
        self.timer_ctrl = self._create_timer_from_config_str(self.timer_ctrl_config_str)
        self.publisher_ctrl = self._create_publisher_from_config_str(self.pub_ctrl_config_str, "ctrl")
        self.subscriber_est = self._create_subscription_from_config_str(self.sub_est_config_str, "est")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the controller."""
        super().on_activate(state)
        self.timer_ctrl.reset()  # activate the control timer
        self.x_hat = None  # reset stateful quantities
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the controller."""
        super().on_deactivate(state)
        self.timer_ctrl.cancel()  # deactivate the control timer
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the controller."""
        super().on_cleanup(state)

        # destroy publishers+timers and subscribers
        self.destroy_timer(self.timer_ctrl)
        self.destroy_publisher(self.publisher_ctrl)
        self.destroy_subscription(self.subscriber_est)
        del self.timer_ctrl
        del self.publisher_ctrl
        del self.subscriber_est

        # delete config strings
        del self.timer_ctrl_config_str
        del self.pub_ctrl_config_str
        del self.sub_est_config_str

        return TransitionCallbackReturn.SUCCESS

    @abstractmethod
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        # TODO(ahl): fill this implementation when we know what the EstimatedState message looks like

    @abstractmethod
    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal.

        This is the control timer callback and is expected to call self.publisher_ctrl internally. Note that the control
        message is still returned afterwards, mostly for logging/debugging purposes. The publish call is the important
        part, NOT the returned value, since the topic is what the ObeliskRobot subscribes to.

        Returns:
            obelisk_control_msg: An Obelisk message type containing the control signal and relevant metadata.
        """
