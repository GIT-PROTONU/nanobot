"""sys.path setup: make web_control importable without a colcon build (pytest's
rootdir jumps to the package dir, skipping the repo-root conftest, when these
tests are run directly)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
