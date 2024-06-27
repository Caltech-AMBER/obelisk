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
        """Initialize the Obelisk estimator."""
        super().__init__(node_name)
        self.declare_parameter("timer_est_setting", rclpy.Parameter.Type.STRING)
        self.declare_parameter("pub_est_setting", rclpy.Parameter.Type.STRING)
        self.declare_parameter("sub_sensor_settings", rclpy.Parameter.Type.STRING_ARRAY)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)

        # parsing config strings
        self.timer_est_setting = self.get_parameter("timer_est_setting").get_parameter_value().string_value
        self.pub_est_setting = self.get_parameter("pub_est_setting").get_parameter_value().string_value
        self.sub_sensor_settings = self.get_parameter("sub_sensor_settings").get_parameter_value().string_array_value

        # create publisher+timer
        self.timer_est = self._create_timer_from_config_str(self.timer_est_setting, self.compute_state_estimate)
        self.publisher_est = self._create_publisher_from_config_str(self.pub_est_setting)
        self.subscriber_sensors = []

        # in the derived class, you must create your own sensor subscribers
        """
        for sensor_setting in self.sub_sensor_settings:
            sub_sensor = self._create_subscription_from_config_str(sensor_setting, self.<sensor_callback>)
            self.subscriber_sensors.append(sub_sensor)
        """

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

        # destroy publishers+timers and subscribers
        self.destroy_timer(self.timer_est)
        self.destroy_publisher(self.publisher_est)
        for sensor_subscriber in self.subscriber_sensors:
            self.destroy_subscription(sensor_subscriber)

        del self.timer_est
        del self.publisher_est
        del self.subscriber_sensors

        # delete config strings
        del self.timer_est_setting
        del self.pub_est_setting
        del self.sub_sensor_settings

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
