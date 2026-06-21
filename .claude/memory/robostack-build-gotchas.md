---
name: robostack-build-gotchas
description: Fixes required to colcon-build ROS 2 Humble (ament_cmake/rosidl) packages under pixi + RoboStack on the NanoPi
metadata: 
  node_type: memory
  type: reference
  originSessionId: 0ebdfb94-1ddc-4321-994d-4ecc12775e00
---

Building a custom `ament_cmake` interfaces package (rosidl) under **pixi + RoboStack Humble** (channel `robostack-staging`, aarch64) needs three fixes, all in `pixi.toml` / `scripts/build.sh` (see [[project-overview]]):

1. **Ninja generator** — the env ships `ninja` but NOT `make`; colcon defaults to "Unix Makefiles" and dies with "CMAKE_MAKE_PROGRAM not set". Set `CMAKE_GENERATOR = "Ninja"` in `[activation.env]`.
2. **Pin `cmake<4`** — env resolves cmake 4.x, but Humble's ament/rosidl CMake modules predate CMake 4's FindPython rewrite. Use `cmake = ">=3.22,<4"` (gets 3.31.x).
3. **Explicit Python hints** — even on cmake 3.31, `rosidl_generator_py`'s `find_package(Python ... COMPONENTS Interpreter Development NumPy)` reports EVERYTHING missing (could not be reproduced in isolation — it's some ament/rosidl state). Fix by pointing FindPython at the conda interpreter via colcon `--cmake-args`: `-DPython_EXECUTABLE`, `-DPython3_EXECUTABLE`, `-DPython_INCLUDE_DIR`, `-DPython_LIBRARY`, `-DPython_NumPy_INCLUDE_DIR` (all under `$CONDA_PREFIX`, numpy path from `numpy.get_include()`). Implemented in `scripts/build.sh`; `pixi run build` calls it.

With these, all 6 packages build. Pure-`ament_python` packages (web_control, oled_display, motor_control, wheel_odometry, robot_bringup) don't need the hints, but robot_msgs does. Conda compilers are `aarch64-conda-linux-gnu-*`; gcc/g++/clang all present.
