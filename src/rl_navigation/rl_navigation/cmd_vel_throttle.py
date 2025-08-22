#!/usr/bin/env python3
# cmd_vel_throttle.py — throttle /cmd_vel_raw to a fixed rate using sim time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

class CmdVelThrottle(Node):
    def __init__(self):
        super().__init__('cmd_vel_throttle')
        # 仿真时钟
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # 可调发布频率（Hz）
        self.declare_parameter('rate', 30.0)
        rate = float(self.get_parameter('rate').value)

        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self._last = None
        self.sub = self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd, qos_in)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw_throttled', 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(f'cmd_vel_throttle: sim time, rate={rate} Hz')

    def _on_cmd(self, msg: Twist):
        self._last = msg

    def _tick(self):
        if self._last is not None:
            self.pub.publish(self._last)

def main():
    rclpy.init()
    node = CmdVelThrottle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
