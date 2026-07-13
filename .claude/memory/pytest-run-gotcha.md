---
name: pytest-run-gotcha
description: "The documented `pixi run python -m pytest src/...` FAILS in this env — launch_testing's pytest plugin is incompatible AND the packages need install/setup.bash sourced; working invocation inside"
metadata: 
  node_type: memory
  type: project
  originSessionId: 4fb1b0bf-a114-4170-8bcc-5c7edbe3368a
---

The test command documented in CLAUDE.md/test docstrings (`pixi run python -m pytest
src/behavior/test`) **does not work as written** (found 2026-07-13):

1. RoboStack's `launch_testing` ships a pytest plugin whose hookimpls are incompatible
   with the env's pytest (`PluginValidationError: Argument(s) {'path'} are declared in
   the hookimpl but can not be found in the hookspec`) — collection dies before any test
   runs. Disabling just that plugin by name isn't enough (a second
   `launch_testing_ros_pytest_entrypoint` plugin then fails on its missing hookspec).
2. The test modules import `behavior` / `web_control` via the colcon egg-links, so the
   workspace must be sourced or imports fail at collection.

**Why:** and **How to apply:** the working invocation is

    pixi run bash -c 'source install/setup.bash 2>/dev/null; \
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/behavior/test src/web_control/test -q'

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` skips every entry-point plugin (none of these
ROS-free unit tests need any), which sidesteps the launch_testing breakage entirely.
Related: [[robostack-build-gotchas]].
