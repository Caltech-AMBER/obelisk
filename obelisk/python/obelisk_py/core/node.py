from typing import Any, Callable, Dict, List, Optional, Tuple, Type, TypeVar, Union

import yaml
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

    Settings
    --------
    Each ObeliskNode receives a single ROS parameter named ``obelisk_settings`` whose value is a YAML
    string holding the node's full configuration: ``publishers``, ``subscribers``, ``timers``, ``sim``,
    and ``callback_groups``. The launch-side producer (``obelisk_py.core.utils.launch_utils``) builds
    that YAML directly from the launch config file, so the structure inside ``obelisk_settings`` is the
    same as the YAML the user wrote — no escape characters, no flat ``field:value,...`` strings, full
    support for nested structures.

    Components are registered by ``key`` via ``register_obk_publisher`` / ``register_obk_subscription``
    / ``register_obk_timer``. At ``on_configure`` time the node walks the parsed settings dict and
    instantiates each registered component from the entry whose ``key`` matches.
    """

    OBK_SETTINGS_PARAM = "obelisk_settings"

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk node."""
        super().__init__(node_name)
        # The single YAML-string parameter that carries all Obelisk-internal settings.
        self.declare_parameter(self.OBK_SETTINGS_PARAM, "")
        # ROS parameter designed to let the user feed a file path for their own code.
        self.declare_parameter("params_path", "")

        # Deferred component registrations — populated by register_obk_*, consumed in on_configure.
        self._obk_pub_settings: List[Dict[str, Any]] = []
        self._obk_sub_settings: List[Dict[str, Any]] = []
        self._obk_timer_settings: List[Dict[str, Any]] = []

        self.obk_callback_groups: Dict[str, CallbackGroup] = {}
        self.obk_publishers: Dict[str, Any] = {}
        self.obk_subscriptions: Dict[str, Any] = {}
        self.obk_timers: Dict[str, Any] = {}

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

    def register_obk_publisher(self, key: str, msg_type: Type) -> None:
        """Register a publisher under ``key``.

        The publisher is created at ``on_configure`` time from the matching entry in
        ``obelisk_settings.publishers`` (the entry whose ``key`` field equals ``key``).

        Parameters:
            key: Identifier used to look up the publisher in ``obelisk_settings.publishers`` and
                in ``self.obk_publishers``.
            msg_type: The message type class.
        """
        self._obk_pub_settings.append({"key": key, "msg_type": msg_type})

    def register_obk_subscription(
        self,
        key: str,
        callback: Callable[[MsgType], None],
        msg_type: Type,
    ) -> None:
        """Register a subscription under ``key``.

        Parameters:
            key: Identifier used to look up the subscription in ``obelisk_settings.subscribers`` and
                in ``self.obk_subscriptions``.
            callback: Callback invoked when a message is received.
            msg_type: The message type class.
        """
        self._obk_sub_settings.append({"key": key, "callback": callback, "msg_type": msg_type})

    def register_obk_timer(
        self,
        key: str,
        callback: Callable[[], Union[Any, Tuple[Any, ...]]],
    ) -> None:
        """Register a timer under ``key``.

        Parameters:
            key: Identifier used to look up the timer in ``obelisk_settings.timers`` and in
                ``self.obk_timers``.
            callback: Callback invoked on each timer tick.
        """
        self._obk_timer_settings.append({"key": key, "callback": callback})

    # ############# #
    # SETTINGS LOAD #
    # ############# #

    def _load_obk_settings(self) -> Dict[str, Any]:
        """Parse the ``obelisk_settings`` ROS parameter into a Python dict."""
        raw = self.get_parameter(self.OBK_SETTINGS_PARAM).get_parameter_value().string_value
        if not raw:
            return {}
        loaded = yaml.safe_load(raw)
        if loaded is None:
            return {}
        if not isinstance(loaded, dict):
            raise ValueError(
                f"obelisk_settings must be a YAML mapping at the top level; got {type(loaded).__name__}."
            )
        return loaded

    @staticmethod
    def _build_callback_groups(spec: Dict[str, str]) -> Dict[str, CallbackGroup]:
        """Build callback groups from a ``{name: type_name}`` mapping."""
        groups: Dict[str, CallbackGroup] = {}
        for name, type_name in spec.items():
            if type_name == "MutuallyExclusiveCallbackGroup":
                groups[name] = MutuallyExclusiveCallbackGroup()
            elif type_name == "ReentrantCallbackGroup":
                groups[name] = ReentrantCallbackGroup()
            else:
                raise ValueError(f"Invalid callback group type: {type_name}")
        return groups

    def _resolve_callback_group(self, entry: Dict[str, Any]) -> Optional[CallbackGroup]:
        """Look up the callback group named in ``entry['callback_group']`` (if any)."""
        cbg_name = entry.get("callback_group")
        if cbg_name is None:
            return None
        if not isinstance(cbg_name, str):
            raise ValueError(f"`callback_group` must be a string, got {type(cbg_name).__name__}.")
        if cbg_name.lower() == "none":
            return None
        cbg = self.obk_callback_groups.get(cbg_name)
        if cbg is None:
            self.get_logger().warn(f"Callback group {cbg_name} not found in node. Using None instead.")
        return cbg

    @staticmethod
    def _find_entry_by_key(entries: List[Dict[str, Any]], key: str) -> Optional[Dict[str, Any]]:
        """Return the first entry in ``entries`` with ``entry['key'] == key``, or None."""
        for entry in entries:
            if entry.get("key") == key:
                return entry
        return None

    # ################### #
    # LIFECYCLE CALLBACKS #
    # ################### #

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node: parse ``obelisk_settings`` and instantiate registered components."""
        super().on_configure(state)

        settings = self._load_obk_settings()

        # Callback groups first — components reference them by name.
        cbg_spec = settings.get("callback_groups", {}) or {}
        self.obk_callback_groups = ObeliskNode._build_callback_groups(cbg_spec)
        for name, cbg in self.obk_callback_groups.items():
            setattr(self, name, cbg)

        pub_entries: List[Dict[str, Any]] = settings.get("publishers", []) or []
        sub_entries: List[Dict[str, Any]] = settings.get("subscribers", []) or []
        timer_entries: List[Dict[str, Any]] = settings.get("timers", []) or []

        # Publishers
        for pub_dict in self._obk_pub_settings:
            key = pub_dict["key"]
            entry = self._find_entry_by_key(pub_entries, key)
            if entry is None:
                self.get_logger().warn(f"Publisher {key} has no entry in obelisk_settings.publishers!")
                continue
            self._create_publisher_from_entry(entry, key=key, msg_type=pub_dict["msg_type"])

        # Subscriptions
        for sub_dict in self._obk_sub_settings:
            key = sub_dict["key"]
            entry = self._find_entry_by_key(sub_entries, key)
            if entry is None:
                self.get_logger().warn(f"Subscription {key} has no entry in obelisk_settings.subscribers!")
                continue
            self._create_subscription_from_entry(
                entry, key=key, msg_type=sub_dict["msg_type"], callback=sub_dict["callback"]
            )

        # Timers
        for timer_dict in self._obk_timer_settings:
            key = timer_dict["key"]
            entry = self._find_entry_by_key(timer_entries, key)
            if entry is None:
                self.get_logger().warn(f"Timer {key} has no entry in obelisk_settings.timers!")
                continue
            self._create_timer_from_entry(entry, key=key, callback=timer_dict["callback"])

        self.get_logger().info(f"{self.get_name()} configured.")
        return TransitionCallbackReturn.SUCCESS

    # ############### #
    # COMPONENT CREATE #
    # ############### #

    def _create_publisher_from_entry(self, entry: Dict[str, Any], key: str, msg_type: Type) -> None:
        if "topic" not in entry:
            raise ValueError(f"Publisher entry for key '{key}' is missing required field 'topic'.")
        topic = entry["topic"]
        if not isinstance(topic, str):
            raise ValueError(f"Publisher '{key}': 'topic' must be a string, got {type(topic).__name__}.")
        history_depth = entry.get("history_depth", 10)
        if not isinstance(history_depth, int):
            raise ValueError(f"Publisher '{key}': 'history_depth' must be an int, got {type(history_depth).__name__}.")

        self.obk_publishers[key] = self.create_publisher(
            msg_type=msg_type,
            topic=topic,
            qos_profile=history_depth,
            callback_group=self._resolve_callback_group(entry),
        )

    def _create_subscription_from_entry(
        self,
        entry: Dict[str, Any],
        key: str,
        msg_type: Type,
        callback: Callable[[MsgType], None],
    ) -> None:
        if "topic" not in entry:
            raise ValueError(f"Subscription entry for key '{key}' is missing required field 'topic'.")
        topic = entry["topic"]
        if not isinstance(topic, str):
            raise ValueError(f"Subscription '{key}': 'topic' must be a string, got {type(topic).__name__}.")
        history_depth = entry.get("history_depth", 10)
        if not isinstance(history_depth, int):
            raise ValueError(
                f"Subscription '{key}': 'history_depth' must be an int, got {type(history_depth).__name__}."
            )

        self.obk_subscriptions[key] = self.create_subscription(
            msg_type=msg_type,
            topic=topic,
            callback=callback,  # type: ignore[arg-type]
            qos_profile=history_depth,
            callback_group=self._resolve_callback_group(entry),
        )

    def _create_timer_from_entry(
        self,
        entry: Dict[str, Any],
        key: str,
        callback: Callable[[], Union[Any, Tuple[Any, ...]]],
    ) -> None:
        if "timer_period_sec" not in entry:
            raise ValueError(f"Timer entry for key '{key}' is missing required field 'timer_period_sec'.")
        period = entry["timer_period_sec"]
        if not isinstance(period, (int, float)):
            raise ValueError(f"Timer '{key}': 'timer_period_sec' must be a number, got {type(period).__name__}.")

        timer = self.create_timer(
            float(period),
            callback=callback,
            callback_group=self._resolve_callback_group(entry),  # type: ignore[arg-type]
        )
        # Timers should start deactivated; they're enabled in on_activate.
        timer.cancel()
        self.obk_timers[key] = timer

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node."""
        super().on_activate(state)
        for timer in self.obk_timers.values():
            timer.reset()
        self.get_logger().info(f"{self.get_name()} activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node."""
        super().on_deactivate(state)
        for timer in self.obk_timers.values():
            timer.cancel()
        self.get_logger().info(f"{self.get_name()} deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the node."""
        super().on_cleanup(state)

        self.obk_callback_groups = {}
        self.obk_publishers = {}
        self.obk_subscriptions = {}
        self.obk_timers = {}

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
        """Shut down the node."""
        super().on_shutdown(state)
        self.on_cleanup(state)
        self.get_logger().info(f"{self.get_name()} shut down.")
        return TransitionCallbackReturn.SUCCESS
