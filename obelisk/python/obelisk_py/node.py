from typing import Callable, Optional, Type, TypeVar, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.subscription import Subscription

from obelisk_py import OBELISK_MSG_TYPES, ROS_ALLOWED_MSG_TYPES
from obelisk_py.exceptions import ObeliskMsgError

ObeliskMsgType = TypeVar("ObeliskMsgType")


class ObeliskNode(LifecycleNode):
    """A class for interfacing with the Obelisk system."""

    def create_publisher(
        self,
        msg_type: Type[ObeliskMsgType],
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[PublisherEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        publisher_class: Type[Publisher] = Publisher,
    ) -> Publisher:
        """Create a new publisher that can only publish Obelisk messages.

        See: github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1242

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        if msg_type not in OBELISK_MSG_TYPES + ROS_ALLOWED_MSG_TYPES:
            raise ObeliskMsgError(
                f"msg_type must be one of {[t.__name__ for t in OBELISK_MSG_TYPES]}. Got {msg_type.__name__}."
            )

        return super().create_publisher(
            msg_type=msg_type,
            topic=topic,
            qos_profile=qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            publisher_class=publisher_class,
        )

    def create_subscription(
        self,
        msg_type: Type[ObeliskMsgType],
        topic: str,
        callback: Callable[[ObeliskMsgType], None],
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False,
    ) -> Subscription:
        """Create a new subscription that can only subscribe to Obelisk messages.

        See: github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1316

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        if msg_type not in OBELISK_MSG_TYPES + ROS_ALLOWED_MSG_TYPES:
            raise ObeliskMsgError(
                f"msg_type must be one of {[t.__name__ for t in OBELISK_MSG_TYPES]}. Got {msg_type.__name__}."
            )

        return super().create_subscription(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
            event_callbacks=event_callbacks,
            qos_overriding_options=qos_overriding_options,
            raw=raw,
        )
