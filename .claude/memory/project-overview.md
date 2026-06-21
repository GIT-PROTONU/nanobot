---
name: project-overview
description: "What the Nano robot project is, its hardware, and the key stack decisions"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0ebdfb94-1ddc-4321-994d-4ecc12775e00
---

Nano = a mobile robot built on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)** running **Armbian**.

Hardware: Roborock **LDS02RR** lidar (serial/UART), quadrature **wheel encoders** (GPIO), **PCA9685** PWM motor driver (I2C), **SSD1306** OLED (I2C), **BWT901CL** 9-axis IMU (WitMotion, USB-serial @115200).

Stack decisions (chosen with the user 2026-06-17):
- Software managed with **pixi + RoboStack** (ROS 2 **Humble** as conda packages, channel `robostack-staging`). No apt.
- Middleware is **`rmw_zenoh`** (not FastDDS) — chosen specifically because RAM is tight on the 1 GB board. Needs `rmw_zenohd` router running.
- **Mixed languages**: rclpy (Python) for the I2C/GPIO nodes, **Rust (`r2r`)** for the LDS driver (the hot path).
- Web UI = **rosbridge + a custom static HTML page** (not Foxglove). The original Axum/Actix idea was dropped.
- ROS-2-style package split under `src/`: robot_msgs, robot_bringup, lds_driver (Rust — currently **doesn't build**, see [[deployment-state]]), lds_driver_py (Python LDS driver actually in use), wheel_odometry, motor_control, oled_display, imu_driver, sys_monitor, web_control.

NOTE: the user first said "split like ROS2 but DON'T use ROS2", then reversed to "use very lightweight ros2" — current truth is **they ARE using ROS 2**. Central config: `src/robot_bringup/config/robot.yaml`. Bus/pin reference: `nanopi-neo-plus2-pinmap.md`.
