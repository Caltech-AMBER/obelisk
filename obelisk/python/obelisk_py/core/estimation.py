from abc import ABC, abstractmethod
from typing import Union

import obelisk_sensor_msgs.msg as osm
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.node import ObeliskNode
from obelisk_py.core.obelisk_typing import ObeliskEstimatorMsg, ObeliskSensorMsg
from obelisk_py.core.utils.internal import get_classes_in_module


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
        self._has_sensor_subscriber = False
        self.register_obk_timer(
            "timer_est_setting",
            self.compute_state_estimate,
            key="timer_est",
        )
        self.register_obk_publisher(
            "pub_est_setting",
            key="pub_est",
            msg_type=None,  # generic, specified in config file
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)

        # ensure there is at least one sensor subscriber
        for sub_dict in self._obk_sub_settings:
            msg_type = sub_dict["msg_type"]
            if msg_type in get_classes_in_module(osm):
                self._has_sensor_subscriber = True
                break
        assert self._has_sensor_subscriber, "At least one sensor subscriber is required in an ObeliskEstimator!"

        return TransitionCallbackReturn.SUCCESS

    @abstractmethod
    def compute_state_estimate(self) -> Union[ObeliskEstimatorMsg, ObeliskSensorMsg]:
        """Compute the state estimate.

        This is the state estimate timer callback and is expected to call 'publisher_est' internally. Note that the
        state estimate message is still returned afterwards, mostly for logging/debugging purposes. The publish call is
        the important part, NOT the returned value, since the topic is what the ObeliskController subscribes to.

        Returns:
            state_estimate: the state estimate message. Can be either an estimator message or, in the case of output
                feedback, a sensor message.
        """
