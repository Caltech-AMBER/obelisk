from abc import ABC, abstractmethod

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.internal_utils import get_classes_in_module
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

        # required parameters
        self.declare_parameter("dt_ctrl", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("msg_type_ctrl", rclpy.Parameter.Type.STRING)
        self.declare_parameter("msg_type_est", rclpy.Parameter.Type.STRING)

        # optional parameters
        self.declare_parameter("history_depth_ctrl", 10)
        self.declare_parameter("history_depth_est", 10)
        self.declare_parameter("cb_group_ctrl", "None")
        self.declare_parameter("cb_group_est", "None")

        # stateful quantities
        self.x_hat = None  # the most recent state estimate

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)

        # ################################### #
        # instantiate required ROS parameters #
        # ################################### #
        # control frequency
        self.dt_ctrl: float = self.get_parameter("dt_ctrl").get_parameter_value().double_value

        # control message type
        _msg_type_ctrl: str = self.get_parameter("msg_type_ctrl").get_parameter_value().string_value
        ocm_type_names = [t.__name__ for t in get_classes_in_module(ocm)]
        assert _msg_type_ctrl in ocm_type_names, f"msg_type_ctrl must be one of {ocm_type_names}"
        for ocm_type_name in ocm_type_names:
            if _msg_type_ctrl == ocm_type_name:
                self.msg_type_ctrl = getattr(ocm, ocm_type_name)
                break

        # state estimate message type
        _msg_type_est: str = self.get_parameter("msg_type_est").get_parameter_value().string_value
        oem_type_names = [t.__name__ for t in get_classes_in_module(oem)]
        assert _msg_type_est in oem_type_names, f"msg_type_est must be one of {oem_type_names}"
        for oem_type_name in oem_type_names:
            if _msg_type_est == oem_type_name:
                self.msg_type_est = getattr(oem, oem_type_name)
                break

        # ################################### #
        # instantiate optional ROS parameters #
        # ################################### #
        # control/state estimate message history depth
        self.history_depth_ctrl: int = self.get_parameter("history_depth_ctrl").get_parameter_value().integer_value
        self.history_depth_est: int = self.get_parameter("history_depth_est").get_parameter_value().integer_value

        # control/state estimate callback groups
        valid_cbg_names = ["ReentrantCallbackGroup", "MutuallyExclusiveCallbackGroup", "None"]
        _cb_group_ctrl_name: str = self.get_parameter("cb_group_ctrl").get_parameter_value().string_value
        _cb_group_est_name: str = self.get_parameter("cb_group_est").get_parameter_value().string_value
        assert _cb_group_ctrl_name in valid_cbg_names, f"cb_group_ctrl must be one of {valid_cbg_names}"
        assert _cb_group_est_name in valid_cbg_names, f"cb_group_est must be one of {valid_cbg_names}"
        self.cb_group_ctrl = None if _cb_group_ctrl_name == "None" else getattr(CallbackGroup, _cb_group_ctrl_name)()
        self.cb_group_est = None if _cb_group_est_name == "None" else getattr(CallbackGroup, _cb_group_est_name)()

        # ##################################### #
        # declare publishers+timers/subscribers #
        # ##################################### #
        # TODO(ahl): should the publisher/timer cb groups always be the same?
        # [NOTE] bad type declaration in rclpy's create_timer definition
        # github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1484
        # PR(ahl): github.com/ros2/rclpy/pull/1306
        self.timer_ctrl = self.create_timer(
            self.dt_ctrl,
            self.compute_control,
            callback_group=self.cb_group_ctrl,  # type: ignore
            # autostart=False,  # TODO(ahl): feature only available after humble
        )
        self.timer_ctrl.cancel()  # initially, the timer should be deactivated, TODO(ahl): remove if distro upgraded
        self.publisher_ctrl = self.create_publisher(
            self.msg_type_ctrl,
            f"/obelisk/{self.node_name}/control",
            self.history_depth_ctrl,
            callback_group=self.cb_group_ctrl,
        )
        self.subscriber_est = self.create_subscription(
            self.msg_type_est,
            f"/obelisk/{self.node_name}/state_estimate",
            self.update_x_hat,
            self.history_depth_est,
            callback_group=self.cb_group_est,
        )
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

        # delete attributes
        del self.dt_ctrl
        del self.msg_type_ctrl
        del self.msg_type_est
        del self.history_depth_ctrl
        del self.history_depth_est
        del self.cb_group_ctrl
        del self.cb_group_est

        # destroy publishers+timers and subscribers
        self.destroy_timer(self.timer_ctrl)
        self.destroy_publisher(self.publisher_ctrl)
        self.destroy_subscription(self.subscriber_est)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shut down the controller."""
        super().on_shutdown(state)
        self.on_cleanup(state)
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

        This is the control timer callback and is expected to call self.publisher_ctrl internally.

        Returns:
            obelisk_control_msg: An Obelisk message type containing the control signal and relevant metadata.
        """
