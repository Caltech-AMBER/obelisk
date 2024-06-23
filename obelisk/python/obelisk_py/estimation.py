from abc import ABC, abstractmethod

import obelisk_estimator_msgs.msg as oem
import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.internal_utils import get_classes_in_module
from obelisk_py.node import ObeliskNode
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg


class ObeliskEstimator(ABC, ObeliskNode):
    """Abstract Obelisk estimator node.

    Obelisk estimators are stateful. That is, all the quantities required to compute the estimate are stored in the
    estimator object itself. This is done because the processes generating each of these quantities may act
    asynchronously. Similarly, the state estimate may be queried asynchronously.

    When implementing a new ObeliskEstimator, the user should declare all quantities required to compute the state
    estimate in the constructor. These quantities should be updated by various update_X methods. Finally, the
    compute_estimate method should be implemented to compute the state estimate using the updated quantities. Note that
    the estimate message should be of type ObeliskEstimatorMsg to be compatible with the Obelisk ecosystem.

    A template for the update functions could be the following:
    ```
    from obelisk_py.obelisk_typing import ObeliskSensorMsg


    def update_X(self, X_msg: ObeliskSensorMsg) -> None:
        # internal logic that extracts quantity X from the message...
        self.X = X  # update the internal state
    ```

    [NOTE] there are no default sensors; the sensor subscriber must be created by the user in the derived class. You can
    follow the example of the estimator configuration to set up each sensor.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk estimator."""
        super().__init__(node_name)

        # required parameters
        self.declare_parameter("dt_est", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("msg_type_est", rclpy.Parameter.Type.STRING)

        # optional parameters
        self.declare_parameter("history_depth_est", 10)
        self.declare_parameter("cb_group_est", "None")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)

        # ################################### #
        # instantiate required ROS parameters #
        # ################################### #
        # estimator frequency
        self.dt_est: float = self.get_parameter("dt_est").get_parameter_value().double_value

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
        self.history_depth_est: int = self.get_parameter("history_depth_est").get_parameter_value().integer_value

        # control/state estimate callback groups
        valid_cbg_names = ["ReentrantCallbackGroup", "MutuallyExclusiveCallbackGroup", "None"]
        _cb_group_est_name: str = self.get_parameter("cb_group_est").get_parameter_value().string_value
        assert _cb_group_est_name in valid_cbg_names, f"cb_group_ctrl must be one of {valid_cbg_names}"
        self.cb_group_est = None if _cb_group_est_name == "None" else getattr(CallbackGroup, _cb_group_est_name)()

        # ##################################### #
        # declare publishers+timers/subscribers #
        # ##################################### #
        # [NOTE] bad type declaration in rclpy's create_timer definition
        # github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1484
        # PR(ahl): github.com/ros2/rclpy/pull/1306
        self.timer_est = self.create_timer(
            self.dt_est,
            self.compute_state_estimate,
            callback_group=self.cb_group_est,  # type: ignore
        )
        self.publisher_est = self.create_publisher(
            self.msg_type_est,
            f"/obelisk/{self.node_name}/state_estimate",
            self.history_depth_est,
            callback_group=self.cb_group_est,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the estimator."""
        super().on_activate(state)
        self.timer_est.reset()  # reset estimator timer
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the estimator."""
        super().on_deactivate(state)
        self.timer_est.cancel()  # deactivate the estimator timer
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the estimator."""
        super().on_cleanup(state)

        # delete attributes
        del self.dt_est
        del self.msg_type_est
        del self.history_depth_est
        del self.cb_group_est

        # destroy publishers+timers and subscribers
        self.destroy_timer(self.timer_est)
        self.destroy_publisher(self.publisher_est)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shut down the estimator."""
        super().on_shutdown(state)
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    @abstractmethod
    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate.

        This is the state estimate timer callback and is expected to call self.publisher_est internally. Note that the
        state estimate message is still returned afterwards, mostly for logging/debugging purposes. The publish call is
        the important part, NOT the returned value, since the topic is what the ObeliskController subscribes to.

        Returns:
            ObeliskEstimatorMsg: the state estimate message.
        """
