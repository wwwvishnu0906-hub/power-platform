"""Entry point for the Python workspace starter."""

import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def hello(name: str = "world") -> str:
    """Return a greeting string for the given name.

    Args:
        name: The name to greet. Defaults to "world".

    Returns:
        A formatted greeting string.
    """
    return f"Hello, {name}!"


def main() -> None:
    """Run the hello world demonstration."""
    message = hello()
    logger.info(message)

    message = hello("Power Platform")
    logger.info(message)


if __name__ == "__main__":
    main()
