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
        self.pub_global = self.create_publisher(Float32, '/scan_min', 10)          # 全局最小（保留）
        self.pub_front  = self.create_publisher(Float32, '/scan_front_min', 10)    # 前方扇区最小（新增）

        # 参数：前方扇区半角（弧度），默认 25°
        self.front_half_angle = math.radians(35.0)

    def cb(self, msg: LaserScan):
        rng_max = msg.range_max if msg.range_max and math.isfinite(msg.range_max) else 10.0
        # 清洗
        vals = [r for r in msg.ranges if r is not None and math.isfinite(r) and r > 0.0]
        gmin = min(vals) if vals else rng_max
        self.pub_global.publish(Float32(data=float(gmin)))

        # 前方扇区：根据 angle_min/angle_increment 取 [-A, +A]
        a_min = msg.angle_min if math.isfinite(msg.angle_min) else -math.pi
        inc   = msg.angle_increment if math.isfinite(msg.angle_increment) and msg.angle_increment > 0 else (2*math.pi/max(1,len(msg.ranges)))
        n     = len(msg.ranges)
        if n == 0:
            self.pub_front.publish(Float32(data=float(rng_max)))
            return

        # 计算下标范围
        half = self.front_half_angle
        # 目标角度范围 [−half, +half]
        i0 = max(0, int(round(( -half - a_min) / inc)))
        i1 = min(n-1, int(round(( +half - a_min) / inc)))
        if i1 < i0:
            i0, i1 = i1, i0

        # 取该扇区的清洗后最小
        sector = []
        for i in range(i0, i1+1):
            r = msg.ranges[i]
            if r is not None and math.isfinite(r) and r > 0.0:
                sector.append(r)
        fmin = min(sector) if sector else rng_max
        self.pub_front.publish(Float32(data=float(fmin)))

def main():
    rclpy.init()
    node = ScanSanitizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
