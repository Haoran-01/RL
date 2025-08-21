#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 静默时长：监控触发后，刹停并在这段时间内不重放旧命令
MUTE_SEC   = 0.30
STALE_SEC  = 0.80   # 多久没新命令就停一次（可选的小保险）
REPUB_HZ   = 10.0   # 重发频率（仅直通时用于保持输出新鲜度）

try:
    from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
    from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
except Exception:
    from std_msgs.msg import String as ErrorMsg
    from std_msgs.msg import String as VerdictMsg

class CmdGuard(Node):
    """极简守卫：平时直通；监控报错时只刹停，等待下一条动作。"""
    def __init__(self):
        super().__init__('cmd_guard_minimal')
        self.sub_cmd     = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.sub_error   = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)
        self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
        self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_cmd_time = 0.0
        self.last_cmd_msg  = None
        self.mute_until    = 0.0
        self.last_repub    = 0.0

        self.timer = self.create_timer(1.0 / REPUB_HZ, self.tick)
        self.get_logger().info('cmd_guard(minimal): pass-through + brake-on-error')

    # —— 直通通道 ——
    def on_cmd(self, msg: Twist):
        now = time.time()
        self.last_cmd_time = now
        self.last_cmd_msg = msg
        if now >= self.mute_until:
            self.pub_out.publish(msg)

    # —— 监控反馈（只做刹停） ——
    def on_error(self, _msg):
        self.get_logger().warn('monitor_error → BRAKE & mute')
        self.pub_out.publish(Twist())          # 立即刹停
        self.mute_until = time.time() + MUTE_SEC
        self.last_cmd_msg = None               # 清空旧命令，避免“续命绕圈”

    def on_verdict(self, msg):
        # 可选：打印当前 verdict，便于调试
        verdict = getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)
        if verdict: self.get_logger().info(f'verdict={verdict}')

    def tick(self):
        # 轻量保险：长时间没新命令就发一次刹停，避免机器人“吃老命令”
        now = time.time()
        if now - self.last_cmd_time > STALE_SEC:
            self.pub_out.publish(Twist())

def main():
    rclpy.init()
    node = CmdGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
