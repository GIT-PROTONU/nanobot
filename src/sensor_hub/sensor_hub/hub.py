"""Single-process host for the light sensor/driver nodes.

Runs imu_driver, sys_monitor, wheel_odometry and lds_driver_py in ONE process under a
single executor instead of four. On the 1 GB H5 each separate rclpy process carries a
full interpreter + rmw baseline (tens of MB each); merging the four reclaims ~100+ MB
of RAM with NO change to topics, rates or behaviour. Each node keeps its own name, so
params load per-name from `--ros-args --params-file robot.yaml` exactly as before, and
its publishers/subscribers/services are unchanged (`/imu_driver/set_parameters` etc. and
the web live-retune sliders all still work). The serial drivers (imu, lds) keep running
their own reader threads — only the timers / subscription / param callbacks share the
one spin thread, which is trivially cheap at these rates.

Connectivity is still observable per device: the web UI reads `/imu/web` (IMU),
`/lds_hz` + the scan stream (lidar) and `/diagnostics` (system) and flags each stale
source — merging the processes doesn't hide a device dropping out.

Trade-off (accepted when this was chosen): the four no longer fail/restart
independently — a fatal error in one callback stops the shared executor. The serial
drivers self-heal (reconnect on their own threads) and the hot paths guard their own
exceptions, so the common failure (a USB/UART device vanishing) is handled without
taking the process down.
"""
import rclpy
from rclpy.executors import SingleThreadedExecutor

from imu_driver.imu_node import ImuNode
from sys_monitor.monitor_node import MonitorNode
from wheel_odometry.encoder_node import EncoderNode
from lds_driver_py.lds_node import LdsNode

# No inter-node dependencies, so construction order is irrelevant.
NODE_CLASSES = (ImuNode, MonitorNode, EncoderNode, LdsNode)


def main():
    rclpy.init()
    nodes = []
    for cls in NODE_CLASSES:
        try:
            nodes.append(cls())
        except Exception as exc:   # one node failing to construct shouldn't sink the rest
            print(f"[sensor_hub] {cls.__name__} init failed: {exc}", flush=True)

    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    print(f"[sensor_hub] hosting {len(nodes)} nodes: "
          f"{', '.join(n.get_name() for n in nodes)}", flush=True)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
