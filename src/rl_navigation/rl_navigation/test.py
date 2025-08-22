#!/usr/bin/env python3
# odom_watcher.py — print pose, detect teleports/time drift
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
import math

TELEPORT_M   = 0.5     # 位置跳变阈值
DRIFT_SEC    = 0.05    # header.stamp 与 节点时钟 的容忍差

def yaw_from_quat(q):
    # geometry_msgs/Quaternion -> yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class OdomWatcher(Node):
    def __init__(self, use_sim_time=True):
        super().__init__('odom_watcher')
        # 可选：跟随仿真时钟（若你的系统启用了 /clock，建议 True）
        # self.declare_parameter('use_sim_time', True)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 50)
        self.last = None  # (t_sim, x, y)
        self.get_logger().info(f'odom_watcher started (use_sim_time={use_sim_time})')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cb(self, msg: Odometry):
        # 取 header 时间（通常是仿真时钟）
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_node = self._now()
        drift = t_node - t_sim  # >0 表示节点钟领先

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # 跳变检测
        tele = ''
        if self.last is not None:
            _, lx, ly = self.last
            dist = math.hypot(x - lx, y - ly)
            if dist > TELEPORT_M:
                tele = f'  <-- TELEPORT? Δpos={dist:.3f} m'
        self.last = (t_sim, x, y)

        # 漂移检测
        drift_note = ''
        if abs(drift) > DRIFT_SEC:
            drift_note = f'  <-- CLOCK DRIFT? (node-sim={drift*1000:.1f} ms)'

        self.get_logger().info(
            f't_sim={t_sim:.3f}  node={t_node:.3f}  drift={drift*1000:.1f}ms{drift_note}\n'
            f'pose: x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°{tele}'
        )

def main():
    rclpy.init()
    node = OdomWatcher(use_sim_time=True)  # 如系统用 /clock，请改 True
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
