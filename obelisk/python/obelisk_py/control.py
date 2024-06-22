from abc import ABC, abstractmethod

from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg


class ObeliskController(ABC):
    """Abstract Obelisk controller class.

    Obelisk controllers are stateful. That is, all the quantities required to compute the control signal are stored in
    the controller object itself. This is done because the processes generating each of these quantities may act
    asynchronously. Similarly, the control action may be queried asynchronously.

    When implementing a new ObeliskController, the user should declare all quantities required to compute the control
    input in the constructor. These quantities should be updated by various update_X methods. Finally, the
    compute_control method should be implemented to compute the control signal using the updated quantities. Note that
    the control message should be of type ObeliskControlMsg to be compatible with the Obelisk ecosystem.
    """

    def __init__(self) -> None:
        """Initialize the Obelisk controller."""
        self.x_hat = None  # the most recent state estimate

    @abstractmethod
    def update_x_hat(self, x_hat: ObeliskEstimatorMsg) -> None:
        """Update the state estimate."""
        # TODO(ahl): fill this implementation when we know what the EstimatedState message looks like

    @abstractmethod
    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal."""
