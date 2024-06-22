from typing import Callable, Optional, Type, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.subscription import Subscription

from obelisk_py.exceptions import ObeliskMsgError
from obelisk_py.obelisk_typing import ObeliskAllowedMsgType


class ObeliskNode(LifecycleNode):
    """A lifecycle node whose publishers and subscribers can only publish and subscribe to Obelisk messages."""

    def create_publisher(
        self,
        msg_type: Type[ObeliskAllowedMsgType],
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
        if not issubclass(msg_type, ObeliskAllowedMsgType):
            raise ObeliskMsgError(
                f"msg_type must be one of {[a.__name__ for a in ObeliskAllowedMsgType.__args__]}. "
                "Got {msg_type.__name__}."
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
        msg_type: Type[ObeliskAllowedMsgType],
        topic: str,
        callback: Callable[[ObeliskAllowedMsgType], None],
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
        if not issubclass(msg_type, ObeliskAllowedMsgType):
            raise ObeliskMsgError(
                f"msg_type must be one of {[a.__name__ for a in ObeliskAllowedMsgType.__args__]}. "
                "Got {msg_type.__name__}."
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
