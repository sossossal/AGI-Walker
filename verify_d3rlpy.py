import sys
try:
    import d3rlpy
    print(f"SUCCESS: d3rlpy version {d3rlpy.__version__} is installed and importable.")
except ImportError as e:
    print(f"FAILURE: Could not import d3rlpy. Error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"FAILURE: An error occurred: {e}")
    sys.exit(1)
