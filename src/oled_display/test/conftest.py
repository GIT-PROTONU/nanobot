"""sys.path setup for offline tests: make this package importable without a
colcon build (pytest's rootdir jumps to the package dir, skipping the repo-root
conftest, when a single package's tests are run directly)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
