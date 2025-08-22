#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import time  # 保留，不再用于计时（仅备用）

# ===== Monitor tunables =====
MUTE_SEC  = 0.20
STALE_SEC = 0.80
REPUB_HZ  = 10.0

TURN_WZ_EARLY_UNMUTE = 0.35
ERROR_COALESCE_S     = 0.10

try:
    from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
    from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
except Exception:
    from std_msgs.msg import String as ErrorMsg
    from std_msgs.msg import String as VerdictMsg

# === 新增：发布累计违规次数 ===
from std_msgs.msg import UInt32  # <-- NEW

class CmdGuard(Node):
    """平时直通；监控报错时仅刹停并短静默，不注入转向。使用仿真时钟。"""
    def __init__(self):
        super().__init__('cmd_guard_minimal')

        # 使用仿真时钟
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except Exception:
            pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # 订阅/发布
        self.sub_cmd     = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.sub_error   = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)
        self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
        self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)

        # === 新增：违规计数发布器 ===
        self.pub_violation_count = self.create_publisher(UInt32, '/monitor_rl/violation_count', 10)  # <-- NEW

        # 状态
        self.last_cmd_time = 0.0
        self.mute_until    = 0.0
        # __init__ 里
        self.last_error_ts = -1e9  # 确保第一条 error 不会被去抖吞掉

        self.violation_count = 0  # <-- NEW: 累计违规计数

        self.timer = self.create_timer(1.0 / REPUB_HZ, self.tick)

        self.get_logger().info('cmd_guard(minimal): pass-through + brake-on-error (sim time)')

    def _now(self) -> float:
        """仿真时钟（秒）。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: Twist):
        now = self._now()
        self.last_cmd_time = now

        # 静默期若收到“明确转向”命令则提前解除静默并转发
        if now < self.mute_until:
            if abs(getattr(msg.angular, "z", 0.0)) >= TURN_WZ_EARLY_UNMUTE:
                self.mute_until = now
                self.pub_out.publish(msg)
            return

        # 正常直通
        self.pub_out.publish(msg)

    def on_error(self, _msg):
        now = self._now()

        # 合并极密集的 error（去抖）
        if (now - self.last_error_ts) < ERROR_COALESCE_S:
            return
        self.last_error_ts = now

        # === 新增：计数 + 发布 ===
        self.violation_count += 1
        self.pub_violation_count.publish(UInt32(data=self.violation_count))
        self.get_logger().warn(f'monitor_error → BRAKE & mute (violations={self.violation_count})')

        # 刹停 + 短静默
        self.pub_out.publish(Twist())
        self.mute_until = now + MUTE_SEC

    def on_verdict(self, msg):
        verdict = getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)
        if verdict:
            self.get_logger().info(f'verdict={verdict}')

    def tick(self):
        # 定期兜底：太久没新命令就发零速
        if self._now() - self.last_cmd_time > STALE_SEC:
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
