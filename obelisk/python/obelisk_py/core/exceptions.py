class ObeliskMsgError(Exception):
    """Exception raised when ObeliskNode publishers/subscribers are not used with Obelisk messages."""

    def __init__(self, msg: str) -> None:
        """Initialize the ObeliskMsgError class."""
        self.msg = msg

    def __str__(self) -> str:
        """Return the error message."""
        return self.msg
