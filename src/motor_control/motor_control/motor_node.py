"""Differential-drive controller: /cmd_vel (geometry_msgs/Twist) -> PCA9685 PWM.

Each motor uses two PCA9685 channels driving an H-bridge in PWM/PWM mode:
forward channel carries the duty when going forward, reverse channel when going
back (the other stays at 0). Re-map channels in robot.yaml to match your wiring.

Includes a watchdog: if no /cmd_vel arrives within `cmd_timeout`, motors stop.
Also (optionally) spins the LDS motor at a constant duty from a spare channel.

Publishes robot_msgs/MotorCommand on /motor_command for introspection.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from robot_msgs.msg import MotorCommand
from motor_control.pca9685 import PCA9685


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.declare_parameters("", [
            ("i2c_bus", 1), ("i2c_address", 0x40), ("pwm_freq_hz", 1000.0),
            ("wheel_separation", 0.16),
            ("max_linear_speed", 0.4), ("max_angular_speed", 3.0),
            ("cmd_timeout", 0.5),
            ("left_fwd_ch", 0), ("left_rev_ch", 1),
            ("right_fwd_ch", 2), ("right_rev_ch", 3),
            ("lds_motor_ch", -1), ("lds_motor_duty", 0.55),
        ])
        g = self.get_parameter
        self.wheel_sep = g("wheel_separation").value
        self.max_lin = g("max_linear_speed").value
        self.max_ang = g("max_angular_speed").value
        self.cmd_timeout = g("cmd_timeout").value
        self.lf, self.lr = g("left_fwd_ch").value, g("left_rev_ch").value
        self.rf, self.rr = g("right_fwd_ch").value, g("right_rev_ch").value
        lds_ch = g("lds_motor_ch").value

        try:
            self.pwm = PCA9685(g("i2c_bus").value, g("i2c_address").value,
                               g("pwm_freq_hz").value)
            self.get_logger().info(
                f"PCA9685 ready on /dev/i2c-{g('i2c_bus').value} "
                f"@0x{g('i2c_address').value:02x}")
        except Exception as exc:
            self.pwm = None
            self.get_logger().error(f"PCA9685 init failed: {exc}. Commands ignored.")

        if self.pwm and lds_ch >= 0:
            self.pwm.set_duty(lds_ch, g("lds_motor_duty").value)
            self.get_logger().info(f"LDS spin motor on ch{lds_ch} "
                                   f"@ {g('lds_motor_duty').value:.0%}")

        self.cmd_pub = self.create_publisher(MotorCommand, "motor_command", 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._last_cmd = self.get_clock().now()
        self.create_timer(0.1, self._watchdog)  # 10 Hz safety check

    def _on_cmd(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        w = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        # differential-drive wheel linear speeds
        vl = v - w * self.wheel_sep * 0.5
        vr = v + w * self.wheel_sep * 0.5
        # normalise to [-1, 1] against max wheel speed
        max_wheel = self.max_lin + self.max_ang * self.wheel_sep * 0.5
        left = vl / max_wheel if max_wheel else 0.0
        right = vr / max_wheel if max_wheel else 0.0
        self._apply(max(-1.0, min(1.0, left)), max(-1.0, min(1.0, right)))

    def _apply(self, left: float, right: float):
        if self.pwm:
            self._drive_side(self.lf, self.lr, left)
            self._drive_side(self.rf, self.rr, right)
        cmd = MotorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.left, cmd.right = float(left), float(right)
        self.cmd_pub.publish(cmd)

    def _drive_side(self, fwd_ch: int, rev_ch: int, duty: float):
        if duty >= 0.0:
            self.pwm.set_duty(rev_ch, 0.0)
            self.pwm.set_duty(fwd_ch, duty)
        else:
            self.pwm.set_duty(fwd_ch, 0.0)
            self.pwm.set_duty(rev_ch, -duty)

    def _watchdog(self):
        dt = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._apply(0.0, 0.0)

    def destroy_node(self):
        if self.pwm:
            self.pwm.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
