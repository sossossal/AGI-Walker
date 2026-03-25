import logging
logger = logging.getLogger(__name__)
import sys


def verify():
    try:
        import d3rlpy

        logger.info(
            f"SUCCESS: d3rlpy version {d3rlpy.__version__} is installed and importable."
        )
    except ImportError as e:
        logger.error(f"FAILURE: Could not import d3rlpy. Error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.info(f"FAILURE: An error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    verify()
