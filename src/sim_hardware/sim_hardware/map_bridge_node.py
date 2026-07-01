"""Republishes slam_nav's /dev/shm/nano_map.bin blob as a real nav_msgs/OccupancyGrid on
/map, purely for RViz (the web UI reads the blob directly and needs no change). Useful on
BOTH the dev-sim and the real robot -- it's not sim-specific, just something nobody needed
until there was an RViz to point at the map.

Blob format (see slam_nav/nav_node.py._write_map + occupancy.py.occupancy_int8): one JSON
metadata line ({"w","h","res","ox","oy",...}), '\n', then row-major int8 cells already in
ROS OccupancyGrid convention (-1 unknown, 0..100 occupied) with row 0 = origin_y -- no
transform needed beyond wrapping it in the message. Published in the "odom" frame: slam_nav
seeds its pose straight from /odom each cycle (see nav_node.py._on_odom), so the grid's
world origin already coincides with odom's origin.
"""
import array
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

MAP_FILE = "/dev/shm/nano_map.bin"


class MapBridgeNode(Node):
    def __init__(self):
        super().__init__("map_bridge")
        self.declare_parameters("", [
            ("map_file", MAP_FILE),
            ("frame_id", "odom"),
            ("publish_rate", 2.0),
        ])
        g = self.get_parameter
        self.map_file = g("map_file").value
        self.frame_id = g("frame_id").value
        self._mtime = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(OccupancyGrid, "map", qos)
        rate = max(0.1, float(g("publish_rate").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"map_bridge up: {self.map_file} -> /map ({self.frame_id})")

    def _tick(self):
        try:
            st = os.stat(self.map_file)
        except OSError:
            return
        if self._mtime == st.st_mtime:
            return          # unchanged since the last publish
        try:
            with open(self.map_file, "rb") as f:
                header = f.readline()
                body = f.read()
        except OSError:
            return
        try:
            meta = json.loads(header)
        except ValueError:
            return
        w, h = int(meta["w"]), int(meta["h"])
        if len(body) < w * h:
            return           # torn read (shouldn't happen -- the writer is atomic)
        self._mtime = st.st_mtime

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(meta["res"])
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = float(meta["ox"])
        msg.info.origin.position.y = float(meta["oy"])
        msg.info.origin.orientation.w = 1.0
        # body bytes are signed int8 (-1 unknown .. 100 occupied); reinterpret rather than
        # treat as unsigned, or -1 round-trips as 255 and RViz renders it as "very occupied".
        msg.data = array.array("b", body[:w * h]).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
