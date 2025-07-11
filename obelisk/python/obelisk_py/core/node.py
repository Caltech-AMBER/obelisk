from typing import Any, Callable, Dict, List, Optional, Tuple, Type, TypeVar, Union

import rclpy
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

MsgType = TypeVar("MsgType")  # hack to denote any message type


class ObeliskNode(LifecycleNode):
    """A lifecycle node that automatically performs component registration according to Obelisk standards.

    By convention, the initialization function should only declare ROS parameters and define stateful quantities.
    Some guidelines for the on_configure, on_activate, and on_deactivate callbacks are provided below.

    Lifecycle Callbacks
    -------------------
    The on_configure callback should do the following:
        * Instantiate required ROS parameters.
        * Instantiate optional ROS parameters.
        * Declare publishers, timers, and subscribers (any timers should be deactivated here).
        * Initialize stateful quantities.

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
        self.declare_parameter("callback_group_settings", "")
        # ROS parameter designed to let the user feed a file path for their own code
        self.declare_parameter("params_path", "")

        # for auto-configuration
        self._obk_pub_settings = []
        self._obk_sub_settings = []
        self._obk_timer_settings = []

        self.obk_callback_groups = {}
        self.obk_publishers = {}
        self.obk_subscriptions = {}
        self.obk_timers = {}

    @property
    def node_name(self) -> str:
        """Get the name of the node."""
        return self.get_name()

    @property
    def t(self) -> float:
        """Get the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # ###################### #
    # COMPONENT REGISTRATION #
    # ###################### #

    def register_obk_publisher(
        self,
        ros_parameter: str,
        msg_type: Type,
        key: Optional[str] = None,
        default_config_str: Optional[str] = None,
    ) -> None:
        """Register a publisher using a configuration string.

        Parameters:
            ros_parameter: The ROS parameter that contains the configuration string.
            msg_type: The message type. If passed, we just use this directly. Otherwise, you can configure this at
                runtime by passing a msg_type field in the config string that corresponds to an Obelisk message type.
            key: The key for the publisher. If None, we look for the key as a field in the config string later.
            default_config_str: The default configuration string. If None, the parameter must be initialized.

        Raises:
            ParameterUninitializedException: If the parameter is uninitialized and no default config string is provided.
        """
        if default_config_str is None:
            self.declare_parameter(ros_parameter, rclpy.Parameter.Type.STRING)
        else:
            self.declare_parameter(ros_parameter, value=default_config_str)
        self._obk_pub_settings.append({"key": key, "ros_parameter": ros_parameter, "msg_type": msg_type})

    def register_obk_subscription(
        self,
        ros_parameter: str,
        callback: Callable[[MsgType], None],
        msg_type: Type,
        key: Optional[str] = None,
        default_config_str: Optional[str] = None,
    ) -> None:
        """Register a subscription using a configuration string.

        Parameters:
            ros_parameter: The ROS parameter that contains the configuration string.
            callback: The callback function.
            msg_type: The message type. If passed, we just use this directly. Otherwise, you can configure this at
                runtime by passing a msg_type field in the config string that corresponds to an Obelisk message type.
            key: The key for the subscription. If None, we look for the key as a field in the config string later.
            default_config_str: The default configuration string. If None, the parameter must be initialized.

        Raises:
            ParameterUninitializedException: If the parameter is uninitialized and no default config string is provided.
        """
        if default_config_str is None:
            self.declare_parameter(ros_parameter, rclpy.Parameter.Type.STRING)
        else:
            self.declare_parameter(ros_parameter, value=default_config_str)
        self._obk_sub_settings.append(
            {"key": key, "ros_parameter": ros_parameter, "callback": callback, "msg_type": msg_type}
        )

    def register_obk_timer(
        self,
        ros_parameter: str,
        callback: Callable[[], Union[Any, Tuple[Any, ...]]],
        key: Optional[str] = None,
        default_config_str: Optional[str] = None,
    ) -> None:
        """Register a timer using a configuration string.

        Parameters:
            ros_parameter: The ROS parameter that contains the configuration string.
            callback: The callback function.
            key: The key for the timer. If None, we look for the key as a field in the config string later.
            default_config_str: The default configuration string. If None, the parameter must be initialized.

        Raises:
            ParameterUninitializedException: If the parameter is uninitialized and no default config string is provided.
        """
        if default_config_str is None:
            self.declare_parameter(ros_parameter, rclpy.Parameter.Type.STRING)
        else:
            self.declare_parameter(ros_parameter, value=default_config_str)
        self._obk_timer_settings.append({"key": key, "ros_parameter": ros_parameter, "callback": callback})

    # ############## #
    # STATIC METHODS #
    # ############## #

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
        if not config_str:
            return [], []

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
            field_names, value_names = list(zip(*[pair.split(":") for pair in field_value_pairs]))  # split by colon
            field_names = list(field_names)  # Convert to list
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
        assert all([field in field_names for field in required_field_names]), (
            f"config_str must contain the following fields: {required_field_names}"
        )
        assert all([field in required_field_names + optional_field_names for field in field_names]), (
            f"""The following fields in the config_str are invalid: {
                set(field_names) - set(required_field_names + optional_field_names)
            }""",
            f"Currently-supported fields in Obelisk are: {required_field_names + optional_field_names}",
        )

    @staticmethod
    def _check_values(value_names: List[str], allowable_value_names: List[str]) -> None:
        """Check if the values in a configuration string are valid.

        Parameters:
            value_names: The list of values.
            allowable_value_names: The allowable values.

        Raises:
            AssertionError: If the values are invalid.
        """
        assert all([value in allowable_value_names for value in value_names]), (
            f"""The following values in the config_str are invalid: {
                set(value_names) - set(allowable_value_names)
            }. """,
            f"Currently-supported values in Obelisk are: {allowable_value_names}",
        )

    @staticmethod
    def _get_key_from_config_dict(config_dict: Dict) -> str:
        """Get the key from a configuration dictionary."""
        assert "key" in config_dict, "No key supplied!"
        assert isinstance(config_dict["key"], str), "The 'key' field must be a string!"
        return config_dict["key"]

    @staticmethod
    def _create_callback_groups_from_config_str(config_str: str) -> Dict[str, CallbackGroup]:
        """Create callback groups from a configuration string.

        Parameters:
            config_str: The list of configuration strings.

        Returns:
            callback_group_dict: A dict whose keys are the callback group names and values are the callback groups.

        Raises:
            ValueError: If the configuration string is invalid.
        """
        # parse and check the configuration string
        field_names, value_names = [], []
        for config in config_str.split(","):
            field_name, value_name = ObeliskNode._parse_config_str(config)
            field_names.extend(field_name)
            value_names.extend(value_name)

        if not field_names and not value_names:
            return {}  # case: no callback groups to create

        allowable_value_names = ["MutuallyExclusiveCallbackGroup", "ReentrantCallbackGroup"]
        ObeliskNode._check_values(value_names, allowable_value_names)
        config_dict = dict(zip(field_names, value_names))

        # create the callback groups
        callback_group_dict = {}
        for callback_group_name, callback_group_type in config_dict.items():
            if callback_group_type == "MutuallyExclusiveCallbackGroup":
                callback_group = MutuallyExclusiveCallbackGroup()
            elif callback_group_type == "ReentrantCallbackGroup":
                callback_group = ReentrantCallbackGroup()
            else:
                raise ValueError(f"Invalid callback group type: {callback_group_type}")
            callback_group_dict[callback_group_name] = callback_group

        return callback_group_dict

    # ####################### #
    # INSTANCE-SPECIFIC UTILS #
    # ####################### #

    def _get_callback_group_from_config_dict(self, config_dict: Dict) -> Optional[CallbackGroup]:
        """Get the callback group from a configuration dictionary."""
        if "callback_group" in config_dict:
            assert isinstance(config_dict["callback_group"], str), "The 'callback_group' field must be a string!"
            if config_dict["callback_group"].lower() == "none":
                return None
            else:
                cbg = self.obk_callback_groups.get(config_dict["callback_group"], None)
                if cbg is None:
                    self.get_logger().warn(
                        f"Callback group {config_dict['callback_group']} not found in node. Using None instead."
                    )
                return cbg
        return None

    def _create_publisher_from_config_str(
        self,
        config_str: str,
        msg_type: Type,
        key: Optional[str] = None,
    ) -> str:
        """Create a publisher from a configuration string and adds it to the publisher dictionary.

        Also creates a key attribute for the publisher. For example, if the key is "pub_ctrl", then the publisher can be
        accessed as self.pub_ctrl.

        [NOTE] There are many unsupported features in this function, such as setting QoS profiles, callback groups, etc.
        For now, we assume the only property of the QoS profile the user will set is history depth, and we don't even
        expose the event_callbacks, qos_overriding_options, or publisher_class parameters.

        Parameters:
            config_str: The configuration string.
            msg_type: The message type. If passed, we just use this directly. Otherwise, you can configure this at
                runtime by passing a msg_type field in the config string that corresponds to an Obelisk message type.
            key: The key for the publisher. If None, we look for the key as a field in the config string.

        Returns:
            key: The key for the publisher.

        Raises:
            AssertionError: If the configuration string is invalid.
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["topic"]
        optional_field_names = ["key", "msg_type", "history_depth", "callback_group"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # parse the key
        if key is None:
            key = ObeliskNode._get_key_from_config_dict(config_dict)
        elif "key" in field_names:
            self.get_logger().warn(
                f"'key'={key} registered for this publisher, and 'key'={config_dict['key']} specified in the config "
                f"string. Using the value 'key'={key}, as hardcoded specifications take precedence!"
            )

        # set the callback group
        callback_group = self._get_callback_group_from_config_dict(config_dict)

        # run type assertions and create the publisher
        history_depth = config_dict.get("history_depth", 10)
        assert isinstance(config_dict["topic"], str), "The 'topic' field must be a string!"
        assert isinstance(history_depth, int), "The 'history_depth' field must be an int!"
        self.obk_publishers[key] = self.create_publisher(
            msg_type=msg_type,
            topic=config_dict["topic"],
            qos_profile=history_depth,
            callback_group=callback_group,
        )
        assert not hasattr(self, key), f"Attribute {key} already exists in the node!"
        setattr(self, key + "_key", self.obk_publishers[key])  # create key attribute for publisher
        return key

    def _create_subscription_from_config_str(
        self,
        config_str: str,
        msg_type: Type,
        callback: Callable[[MsgType], None],
        key: Optional[str] = None,
    ) -> str:
        """Create a subscription from a configuration string and adds it to the subscription dictionary.

        Also creates a key attribute for the subscription. For example, if the key is "sub_ctrl", then the subscription
        can be accessed as self.sub_ctrl.

        [NOTE] There are many unsupported features in this function, such as setting QoS profiles, callback groups, etc.
        For now, we assume the only property of the QoS profile the user will set is history depth, and we don't even
        expose the event_callbacks, qos_overriding_options, or raw parameters.

        Parameters:
            config_str: The configuration string.
            msg_type: The message type. If passed, we just use this directly. Otherwise, you can configure this at
                runtime by passing a msg_type field in the config string that corresponds to an Obelisk message type.
            callback: The callback function.
            key: The key for the subscription. If None, we look for the key as a field in the config string.

        Returns:
            key: The key for the subscription.

        Raises:
            AssertionError: If the configuration string is invalid.
            ObeliskMsgError: If the message type is not an Obelisk message.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["topic"]
        optional_field_names = ["key", "msg_type", "history_depth", "callback_group"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # parse the key
        if key is None:
            key = ObeliskNode._get_key_from_config_dict(config_dict)
        elif "key" in field_names:
            self.get_logger().warn(
                f"'key'={key} registered for this subscription, and 'key'={config_dict['key']} specified in the config "
                f"string. Using the value 'key'={key}, as hardcoded specifications take precedence!"
            )

        # set the callback group
        callback_group = self._get_callback_group_from_config_dict(config_dict)

        # run type assertions and return the subscription
        history_depth = config_dict.get("history_depth", 10)
        assert isinstance(config_dict["topic"], str), "The 'topic' field must be a string!"
        assert isinstance(history_depth, int), "The 'history_depth' field must be an int!"

        self.obk_subscriptions[key] = self.create_subscription(
            msg_type=msg_type,
            topic=config_dict["topic"],
            callback=callback,  # type: ignore
            qos_profile=history_depth,
            callback_group=callback_group,
        )
        assert not hasattr(self, key), f"Attribute {key} already exists in the node!"
        setattr(self, key + "_key", self.obk_subscriptions[key])  # create key attribute for subscription
        return key

    def _create_timer_from_config_str(
        self,
        config_str: str,
        callback: Callable[[], Union[Any, Tuple[Any, ...]]],
        key: Optional[str] = None,
    ) -> str:
        """Create a timer from a configuration string and adds it to the timer dictionary.

        Also creates a key attribute for the timer. For example, if the key is "timer_ctrl", then the timer can be
        accessed as self.timer_ctrl.

        Parameters:
            config_str: The configuration string.
            callback: The callback function.
            key: The key for the timer. If None, we look for the key as a field in the config string.

        Returns:
            key: The key for the timer.

        Raises:
            AssertionError: If the configuration string is invalid.
        """
        # parse and check the configuration string
        field_names, value_names = ObeliskNode._parse_config_str(config_str)
        required_field_names = ["timer_period_sec"]
        optional_field_names = ["key", "callback_group"]
        ObeliskNode._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # parse the key
        if key is None:
            key = ObeliskNode._get_key_from_config_dict(config_dict)
        elif "key" in field_names:
            self.get_logger().warn(
                f"'key'={key} registered for this timer, and 'key'={config_dict['key']} specified in the config "
                f"string. Using the value 'key'={key}, as hardcoded specifications take precedence!"
            )

        # set the callback group
        callback_group = self._get_callback_group_from_config_dict(config_dict)

        # run type assertions and return the timer
        assert isinstance(config_dict["timer_period_sec"], (int, float)), (
            "The 'timer_period_sec' field must be a number!"
        )

        timer = self.create_timer(
            config_dict["timer_period_sec"],
            callback=callback,
            callback_group=callback_group,  # type: ignore
            # autostart=False,  # TODO(ahl): feature only available after humble
        )
        timer.cancel()  # initially, the timer should be deactivated, TODO(ahl): remove if distro upgraded
        self.obk_timers[key] = timer
        assert not hasattr(self, key), f"Attribute {key} already exists in the node!"
        setattr(self, key + "_key", self.obk_timers[key])  # create key attribute for timer
        return key

    # ################### #
    # LIFECYCLE CALLBACKS #
    # ################### #

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node.

        Raises:
            AssertionError: If the configuration string is invalid.
        """
        super().on_configure(state)

        # parsing config strings
        callback_group_settings = self.get_parameter("callback_group_settings").get_parameter_value().string_value

        # create callback groups
        self.obk_callback_groups = ObeliskNode._create_callback_groups_from_config_str(callback_group_settings)
        for callback_group_name, callback_group in self.obk_callback_groups.items():
            setattr(self, callback_group_name, callback_group)

        # create components
        for pub_dict in self._obk_pub_settings:
            key = pub_dict["key"]
            ros_parameter = pub_dict["ros_parameter"]
            msg_type = pub_dict["msg_type"]

            pub_config_str = self.get_parameter(ros_parameter).get_parameter_value().string_value
            if pub_config_str == "":
                self.get_logger().warn(f"Publisher {key} has no configuration string!")
                continue
            final_key = self._create_publisher_from_config_str(pub_config_str, key=key, msg_type=msg_type)
            pub_dict["key"] = final_key  # if no key passed, use value from config file

        for sub_dict in self._obk_sub_settings:
            key = sub_dict["key"]
            ros_parameter = sub_dict["ros_parameter"]
            msg_type = sub_dict["msg_type"]
            callback = sub_dict["callback"]

            sub_config_str = self.get_parameter(ros_parameter).get_parameter_value().string_value
            if sub_config_str == "":
                self.get_logger().warn(f"Subscription {key} has no configuration string!")
                continue
            final_key = self._create_subscription_from_config_str(
                sub_config_str, callback=callback, key=key, msg_type=msg_type
            )
            sub_dict["key"] = final_key  # if no key passed, use value from config file

        for timer_dict in self._obk_timer_settings:
            key = timer_dict["key"]
            ros_parameter = timer_dict["ros_parameter"]
            callback = timer_dict["callback"]

            timer_config_str = self.get_parameter(ros_parameter).get_parameter_value().string_value
            if timer_config_str == "":
                self.get_logger().warn(f"Timer {key} has no configuration string!")
                continue
            final_key = self._create_timer_from_config_str(timer_config_str, callback=callback, key=key)
            timer_dict["key"] = final_key  # if no key passed, use value from config file

        self.get_logger().info(f"{self.get_name()} configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node."""
        super().on_activate(state)
        for timer in self.obk_timers.values():
            timer.reset()  # activate timers

        self.get_logger().info(f"{self.get_name()} activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node."""
        super().on_deactivate(state)
        for timer in self.obk_timers.values():
            timer.cancel()  # deactivate timers

        self.get_logger().info(f"{self.get_name()} deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the node."""
        super().on_cleanup(state)

        # reset internal dicts
        self.obk_callback_groups = {}
        self.obk_publishers = {}
        self.obk_subscriptions = {}
        self.obk_timers = {}

        # destroy publishers, timers, and subscriptions
        if self.timers is not None:
            for timer in self.timers:
                self.destroy_timer(timer)

        if self.publishers is not None:
            for publisher in self.publishers:
                self.destroy_publisher(publisher)

        if self.subscriptions is not None:
            for subscription in self.subscriptions:
                self.destroy_subscription(subscription)

        self.get_logger().info(f"{self.get_name()} cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shut down the controller."""
        super().on_shutdown(state)
        self.on_cleanup(state)
        self.get_logger().info(f"{self.get_name()} shut down.")
        return TransitionCallbackReturn.SUCCESS
