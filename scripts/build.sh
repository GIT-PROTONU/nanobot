#!/usr/bin/env bash
# Build the colcon workspace inside the pixi/RoboStack env.
#
# Why the explicit -DPython_* hints: in this RoboStack Humble env, ament/rosidl's
# CMake chain (python_cmake_module's legacy FindPythonInterp/Libs followed by a
# modern find_package(Python ... Development NumPy)) leaves FindPython unable to
# locate the conda Python by itself, so rosidl_generator_py fails to configure.
# Pointing FindPython straight at the env interpreter (it then derives include/
# lib/numpy by querying it) makes it deterministic. Passed via colcon --cmake-args.
set -euo pipefail

PYVER="$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
NUMPY_INC="$(python -c 'import numpy; print(numpy.get_include())')"

exec colcon build --symlink-install --event-handlers console_direct+ \
  --cmake-args \
    -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python${PYVER}" \
    -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python${PYVER}" \
    -DPython_INCLUDE_DIR="$CONDA_PREFIX/include/python${PYVER}" \
    -DPython_LIBRARY="$CONDA_PREFIX/lib/libpython${PYVER}.so" \
    -DPython_NumPy_INCLUDE_DIR="$NUMPY_INC" \
    -Wno-dev \
  "$@"
