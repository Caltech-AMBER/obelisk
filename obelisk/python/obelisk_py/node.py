from typing import Callable, List, Optional, Tuple, Type, Union, get_args, get_origin

from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from obelisk_py.exceptions import ObeliskMsgError
from obelisk_py.obelisk_typing import ObeliskAllowedMsg, ObeliskMsg, is_in_bound
from obelisk_py.utils import check_and_add_obelisk_msg_attr


class ObeliskNode(LifecycleNode):
    """A lifecycle node whose publishers and subscribers can only publish and subscribe to Obelisk messages.

    By convention, the initialization function should only declare ROS parameters and define stateful quantities.
    Some guidelines for the on_configure, on_activate, and on_deactivate callbacks are provided below.

    Lifecycle Callbacks
    -------------------
    The on_configure callback should do the following:
        * Instantiate required ROS parameters.
        * Instantiate optional ROS parameters.
        * Declare publishers, timers, and subscribers (any timers should be deactivated here).

    The on_activate callback should do the following:
        * Activate any timers that were declared in on_configure (by calling reset()).
        * Resetting any variables or stateful quantities that need particular initial values upon activation.

    The on_deactivate callback should do the following:
        * Deactivate any timers that were activated in on_activate (by calling cancel()).

    The on_cleanup callback should do the following:
        * Clean up any resources that were allocated in on_configure.

    The on_shutdown callback should do the following:
        * Also perform clean up. The main difference is that if shutting down, the node cannot reactivate.

    Configuration Strings
    ---------------------
    The configuration of publishers, subscribers, and timers are done via a concept called configuration strings. A
    configuration string is formatted as follows:
        "field1:value1,field2:value2,...,fieldN:valueN".
    That is, each field-value pair is separated by a comma, and the field and value are separated by a colon. This
    lets the user specify the configuration of the publisher in a single string, which is a compressed way to
    configure publishers or subscribers using the data types available to ROS2 parameters. This also means the user
    can omit fields in favor of default values if desired or pass fields in any order.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk node."""
        super().__init__(node_name)

    @property
    def node_name(self) -> str:
        """Get the name of the node."""
        return self.get_name()

    @staticmethod
    def _parse_config_str(config_str: str) -> Tuple[List[str], List[Union[str, float, int]]]:
        """Parse a configuration string into a list of field names and a list of values.

        Parameters:
            config_str: The configuration string.

        Returns:
            field_names: A list of field names.
            value_names: A list of values.

        Raises:
            ValueError: If the configuration string is invalid.
        """
        assert config_str, "config_str must not be empty!"
        config_str = config_str.replace(" ", "")  # remove all spaces
        config_str = config_str.replace("\n", "")  # remove all newlines
        field_value_pairs = config_str.split(",")  # split by comma

        def _convert_values(value: str) -> Union[str, float, int]:
            """Convert a value to an int or float if possible (helper function)."""
            if value.isdigit():
                return int(value)
            try:
                return float(value)
            except ValueError:
                return value

        try:
            field_names, value_names = zip(*[pair.split(":") for pair in field_value_pairs])  # split by colon
            value_names = [_convert_values(value) for value in value_names]  # convert ints/floats
        except ValueError as e:
            raise ValueError(
                "Each field-value pair in the configuration string must be separated by a colon!\n"
                f"Original stacktrace: {e}"
            ) from e

        return field_names, value_names

    @staticmethod
    def _check_fields(field_names: List[str], required_field_names: List[str], optional_field_names: List[str]) -> None:
        """Check if a configuration string is valid.

        Parameters:
            config_str: The configuration string.
            required_field_names: The required field names.
            optional_field_names: The optional field names.

        Raises:
            AssertionError: If the configuration string is invalid.
        """
        assert all(
            [field in field_names for field in required_field_names]
        ), f"config_str must contain the following fields: {required_field_names}"
        assert all([field in required_field_names + optional_field_names for field in field_names]), (
            f"""The following fields in the config_str are invalid: {
                set(field_names) - set(required_field_names + optional_field_names)
            }""",
            "Currently-supported fields in Obelisk are: {required_field_names + optional_field_names}",
        )

    def _create_publisher_from_config_str(self, config_str: str, msg_attr_suffix: str) -> Publisher:
        """Create a publisher from a configuration string.

        [NOTE] There are many unsupported features in this function, such as setting QoS profiles, callback groups, etc.
        For now, we assume the only property of the QoS profile the user will set is history depth, and we don't even
        expose the event_callbacks, qos_overriding_options, or publisher_class parameters.

        Parameters:
            config_str: The configuration string.
            msg_attr_suffix: The suffix to append to the attribute name when adding the Obelisk message attribute.

        Returns:
            publisher: The created publisher.

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["msg_type", "topic"]
        optional_field_names = ["history_depth", "callback_group", "non_obelisk"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # create the publisher
        if "non_obelisk" not in config_dict or config_dict["non_obelisk"].lower() != "true":
            check_and_add_obelisk_msg_attr(self, config_dict["msg_type"], ObeliskMsg, msg_attr_suffix)
        else:
            self.get_logger().warn(
                "Creating a publisher that can publish non-Obelisk messages. "
                "This may cause certain API incompatibilities."
            )
            check_and_add_obelisk_msg_attr(self, config_dict["msg_type"], ObeliskAllowedMsg, msg_attr_suffix)

        callback_group_name = config_dict.get("callback_group", "None")
        if callback_group_name.lower() == "none":
            callback_group = None
        elif hasattr(self, callback_group_name):
            callback_group = getattr(self, callback_group_name)
        else:
            self.get_logger().warn(f"Callback group {callback_group_name} not found in node. Using None instead.")
            callback_group = None

        return self.create_publisher(
            msg_type=getattr(self, f"msg_type_{msg_attr_suffix}"),
            topic=config_dict["topic"],
            qos_profile=config_dict.get("history_depth", 10),
            callback_group=callback_group,
            non_obelisk=config_dict.get("non_obelisk", "False").lower() == "true",
        )

    def _create_subscription_from_config_str(self, config_str: str, msg_attr_suffix: str) -> Subscription:
        """Create a subscription from a configuration string.

        [NOTE] There are many unsupported features in this function, such as setting QoS profiles, callback groups, etc.
        For now, we assume the only property of the QoS profile the user will set is history depth, and we don't even
        expose the event_callbacks, qos_overriding_options, or raw parameters.

        Parameters:
            config_str: The configuration string.
            msg_attr_suffix: The suffix to append to the attribute name when adding the Obelisk message attribute.

        Returns:
            subscription: The created subscription.

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["msg_type", "topic", "callback"]
        optional_field_names = ["history_depth", "callback_group", "non_obelisk"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # create the subscription
        if "non_obelisk" not in config_dict or config_dict["non_obelisk"].lower() != "true":
            check_and_add_obelisk_msg_attr(self, config_dict["msg_type"], ObeliskMsg, msg_attr_suffix)
        else:
            self.get_logger().warn(
                "Creating a publisher that can publish non-Obelisk messages. "
                "This may cause certain API incompatibilities."
            )
            check_and_add_obelisk_msg_attr(self, config_dict["msg_type"], ObeliskAllowedMsg, msg_attr_suffix)

        callback_group_name = config_dict.get("callback_group", "None")
        if callback_group_name.lower() == "none":
            callback_group = None
        elif hasattr(self, callback_group_name):
            callback_group = getattr(self, callback_group_name)
        else:
            self.get_logger().warn(f"Callback group {callback_group_name} not found in node. Using None instead.")
            callback_group = None

        return self.create_subscription(
            msg_type=getattr(self, f"msg_type_{msg_attr_suffix}"),
            topic=config_dict["topic"],
            callback=getattr(self, config_dict["callback"]),
            qos_profile=config_dict.get("history_depth", 10),
            callback_group=callback_group,
            non_obelisk=config_dict.get("non_obelisk", "False").lower() == "true",
        )

    def _create_timer_from_config_str(self, config_str: str) -> Timer:
        """Create a timer from a configuration string.

        Parameters:
            config_str: The configuration string.

        Returns:
            timer: The created timer.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["timer_period_sec", "callback"]
        optional_field_names = ["callback_group"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # create the timer
        callback_group_name = config_dict.get("callback_group", "None")
        if callback_group_name.lower() == "none":
            callback_group = None
        elif hasattr(self, callback_group_name):
            callback_group = getattr(self, callback_group_name)
        else:
            self.get_logger().warn(f"Callback group {callback_group_name} not found in node. Using None instead.")
            callback_group = None

        callback_group: Optional[CallbackGroup]

        timer = self.create_timer(
            config_dict["timer_period_sec"],
            callback=getattr(self, config_dict["callback"]),
            callback_group=callback_group,  # type: ignore
            # autostart=False,  # TODO(ahl): feature only available after humble
        )
        timer.cancel()  # initially, the timer should be deactivated, TODO(ahl): remove if distro upgraded
        return timer

    def create_publisher(
        self,
        msg_type: ObeliskAllowedMsg,
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[PublisherEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        publisher_class: Type[Publisher] = Publisher,
        non_obelisk: bool = False,
    ) -> Publisher:
        """Create a new publisher that can only publish Obelisk messages.

        See: github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1242

        Parameters:
            non_obelisk: If True, the publisher can publish non-Obelisk messages. Default is False.

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        if not non_obelisk and not is_in_bound(msg_type, ObeliskAllowedMsg):
            if get_origin(ObeliskAllowedMsg.__bound__) is Union:
                valid_msg_types = [a.__name__ for a in get_args(ObeliskAllowedMsg.__bound__)]
            else:
                valid_msg_types = [ObeliskAllowedMsg.__name__]
            raise ObeliskMsgError(
                f"msg_type must be one of {valid_msg_types}. "
                "Got {msg_type.__name__}. If you are sure that the message type is correct, "
                "set non_obelisk=True. Note that this may cause certain API incompatibilies."
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
        msg_type: ObeliskAllowedMsg,
        topic: str,
        callback: Callable[[ObeliskAllowedMsg], None],
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False,
        non_obelisk: bool = False,
    ) -> Subscription:
        """Create a new subscription that can only subscribe to Obelisk messages.

        See: github.com/ros2/rclpy/blob/e4042398d6f0403df2fafdadbdfc90b6f6678d13/rclpy/rclpy/node.py#L1316

        Parameters:
            non_obelisk: If True, the subscriber can receive non-Obelisk messages. Default is False.

        Raises:
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        if not non_obelisk and not is_in_bound(msg_type, ObeliskAllowedMsg):
            if get_origin(ObeliskAllowedMsg.__bound__) is Union:
                valid_msg_types = [a.__name__ for a in get_args(ObeliskAllowedMsg.__bound__)]
            else:
                valid_msg_types = [ObeliskAllowedMsg.__name__]
            raise ObeliskMsgError(
                f"msg_type must be one of {valid_msg_types}. "
                "Got {msg_type.__name__}. If you are sure that the message type is correct, "
                "set non_obelisk=True. Note that this may cause certain API incompatibilies."
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
