from abc import ABC, abstractmethod
from typing import Union

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.node import ObeliskNode
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg, ObeliskSensorMsg


class ObeliskEstimator(ABC, ObeliskNode):
    """Abstract Obelisk estimator node.

    Obelisk estimators are stateful. That is, all the quantities required to compute the estimate are stored in the
    estimator object itself. This is done because the processes generating each of these quantities may act
    asynchronously. Similarly, the state estimate may be queried asynchronously.

    When implementing a new ObeliskEstimator, the user should declare all quantities required to compute the state
    estimate in on_configure. These quantities should be updated by various update_X methods. Finally, the
    compute_estimate method should be implemented to compute the state estimate using the updated quantities. Note that
    the estimate message should be of type ObeliskEstimatorMsg to be compatible with the Obelisk ecosystem.

    A template for the update functions could be the following:
    ```
    from obelisk_py.obelisk_typing import ObeliskSensorMsg


    def update_X(self, X_msg: ObeliskSensorMsg) -> None:
        # internal logic that extracts quantity X from the message...
        self.X = X  # update the internal state
    ```
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk estimator.

        [NOTE] In derived classes, you should declare settings for sensor subscribers.
        """
        super().__init__(node_name)
        self.declare_parameter("timer_est_setting", rclpy.Parameter.Type.STRING)
        self.declare_parameter("pub_est_setting", rclpy.Parameter.Type.STRING)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)

        # parsing config strings
        self.timer_est_setting = self.get_parameter("timer_est_setting").get_parameter_value().string_value
        self.pub_est_setting = self.get_parameter("pub_est_setting").get_parameter_value().string_value

        # create publisher+timer
        self.timer_est = self._create_timer_from_config_str(self.timer_est_setting, self.compute_state_estimate)
        self.publisher_est = self._create_publisher_from_config_str(self.pub_est_setting)

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

    @abstractmethod
    def compute_state_estimate(self) -> Union[ObeliskEstimatorMsg, ObeliskSensorMsg]:
        """Compute the state estimate.

        This is the state estimate timer callback and is expected to call self.publisher_est internally. Note that the
        state estimate message is still returned afterwards, mostly for logging/debugging purposes. The publish call is
        the important part, NOT the returned value, since the topic is what the ObeliskController subscribes to.

        Returns:
            state_estimate: the state estimate message. Can be either an estimator message or, in the case of output
                feedback, a sensor message.
        """
