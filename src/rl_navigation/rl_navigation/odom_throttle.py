#!/usr/bin/env python3
# odom_throttle.py — downsample /odom to a fixed rate using sim time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

class OdomThrottle(Node):
    def __init__(self):
        super().__init__('odom_throttle')
        # 用仿真时钟
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # 可调发布频率（Hz）
        self.declare_parameter('rate', 80.0)
        rate = float(self.get_parameter('rate').value)

        # 订阅 QoS：KEEP_LAST=1 + BEST_EFFORT，避免堆积
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self._last = None
        self.sub = self.create_subscription(Odometry, '/odom', self._on_odom, qos_in)
        self.pub = self.create_publisher(Odometry, '/odom_throttled', 10)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'odom_throttle: sim time, rate={rate} Hz')

    def _on_odom(self, msg: Odometry):
        self._last = msg  # 只留最近一条

    def _tick(self):
        if self._last is not None:
            self.pub.publish(self._last)

def main():
    rclpy.init()
    node = OdomThrottle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
