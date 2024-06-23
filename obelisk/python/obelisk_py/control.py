from abc import ABC, abstractmethod
from typing import Optional, Type

from rclpy.callback_groups import CallbackGroup

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

    def __init__(
        self,
        node_name: str,
        msg_type_ctrl: Type[ObeliskControlMsg],
        cb_group_ctrl: Optional[CallbackGroup] = None,
    ) -> None:
        """Initialize the Obelisk controller."""
        super().__init__(node_name)

        # declare ROS parameters
        self.declare_parameter("dt_ctrl")  # no default value: must be set by the user

        # instantiate ROS parameters
        self.dt_ctrl: float = self.get_parameter("dt_ctrl").get_parameter_value().double_value

        # declare publishers+timers/subscribers
        # TODO(ahl): should these cb groups always be the same?
        # [NOTE] bad type declaration in rclpy's create_timer definition
        # github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1484
        # PR(ahl): github.com/ros2/rclpy/pull/1306
        self.timer_ctrl = self.create_timer(
            self.dt_ctrl,
            self.compute_control,
            callback_group=cb_group_ctrl,  # type: ignore
        )
        self.publisher_ctrl = self.create_publisher(msg_type_ctrl, "control", 10, callback_group=cb_group_ctrl)

        # stateful quantities
        self.x_hat = None  # the most recent state estimate

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
