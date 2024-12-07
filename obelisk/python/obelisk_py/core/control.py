from abc import ABC, abstractmethod

from obelisk_py.core.node import ObeliskNode


class ObeliskController(ABC, ObeliskNode):
    """Abstract Obelisk controller node.

    Obelisk controllers are stateful. That is, all the quantities required to compute the control signal are stored in
    the controller object itself. This is done because the processes generating each of these quantities may act
    asynchronously. Similarly, the control action may be queried asynchronously.

    When implementing a new ObeliskController, the user should declare all quantities required to compute the control
    input in on_configure. These quantities should be updated by various update_X methods. Finally, the
    compute_control method should be implemented to compute the control signal using the updated quantities. Note that
    the control message should be of type ObeliskControlMsg to be compatible with the Obelisk ecosystem.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk controller."""
        super().__init__(node_name)
        self.register_obk_timer(
            "timer_ctrl_setting",
            self.compute_control,
            key="timer_ctrl",
        )
        self.register_obk_publisher(
            "pub_ctrl_setting",
            key="pub_ctrl",
            msg_type=None,  # generic, specified in config file
        )
        self.register_obk_subscription(
            "sub_est_setting",
            self.update_x_hat,
            key="sub_est",
            msg_type=None,  # generic, specified in config file
        )

    @abstractmethod
    def update_x_hat(self, x_hat_msg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """

    @abstractmethod
    def compute_control(self):
        """Compute the control signal.

        This is the control timer callback and is expected to call 'publisher_ctrl' internally. Note that the control
        message is still returned afterwards, mostly for logging/debugging purposes. The publish call is the important
        part, NOT the returned value, since the topic is what the ObeliskRobot subscribes to.

        Returns:
            obelisk_control_msg: An Obelisk message type containing the control signal and relevant metadata.
        """
