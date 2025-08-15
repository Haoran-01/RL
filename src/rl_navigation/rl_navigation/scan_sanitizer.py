#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ScanSanitizer(Node):
    def __init__(self):
        super().__init__('scan_sanitizer')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(Float32, '/scan_min', 10)
    def cb(self, msg: LaserScan):
        rng_max = msg.range_max if msg.range_max and math.isfinite(msg.range_max) else 10.0
        vals = [r for r in msg.ranges if r is not None and math.isfinite(r) and r > 0.0]
        m = min(vals) if vals else rng_max
        self.pub.publish(Float32(data=float(m)))

def main():
    rclpy.init()
    node = ScanSanitizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
