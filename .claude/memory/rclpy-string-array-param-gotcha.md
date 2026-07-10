---
name: rclpy-string-array-param-gotcha
description: rclpy declare_parameter with an empty-list default for a STRING_ARRAY param breaks type inference when a YAML params-file override is also empty
metadata: 
  node_type: memory
  type: reference
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

Discovered 2026-07-10 while building [[scheduled-routines]]. Symptom: `node.declare_parameter("x", [], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))` followed by `node.get_parameter("x")` raises `ParameterUninitializedException`, or (if the YAML params-file override is also present but non-empty) `InvalidParameterTypeException: ... expecting type 'BYTE_ARRAY'`.

**Root cause:** rclpy infers the parameter's array subtype from the Python default value *before* fully honoring the `ParameterDescriptor.type` hint. An empty Python list `[]` has no element to infer a type from, so rclpy silently falls back to (or half-commits to) `PARAMETER_BYTE_ARRAY`, which then conflicts with a non-empty YAML override of a different element type (e.g. strings) — or, if the YAML override is also `[]`, the parameter just never gets a concrete type and reads back as uninitialized.

**Fix:** never default a ROS2 array param to a genuinely empty list, even with an explicit `ParameterDescriptor`. Default to a list with one sentinel element of the right type (e.g. `[""]` for a string array) and treat that single blank entry as "empty" in your own parsing code. Verified fix: `declare_parameter("schedule_times", [""], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))` combined with a `robot.yaml` override of `[""]` resolves correctly to `['']`.

This was ultimately sidestepped entirely in the scheduled-routines feature by moving away from ROS2 params to a JSON file for anything beyond scalars — see [[scheduled-routines]] and the broader ROS2-scope discussion in that session (ROS2 stays for typed control-plane + TF/sim interop; anything structured/nested lives outside it, in JSON files, by design in this codebase).
