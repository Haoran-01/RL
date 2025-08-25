#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

# ===== Monitor tunables =====
MUTE_SEC  = 0.10   # 缩短静默，减小“被按住”的体感
STALE_SEC = 0.80
REPUB_HZ  = 10.0

TURN_WZ_EARLY_UNMUTE = 0.25
ERROR_COALESCE_S     = 0.12

# 兼容 rosmonitoring_interfaces，不在就退到 std_msgs/String
try:
    from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
    from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
except Exception:
    from std_msgs.msg import String as ErrorMsg
    from std_msgs.msg import String as VerdictMsg

from std_msgs.msg import UInt32, Int32MultiArray

class CmdGuard(Node):
    """平时直通；监控报错时刹停并短静默；提供 step 握手确保当步对齐。"""
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
        self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)

        self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_violation_count = self.create_publisher(UInt32, '/monitor_rl/violation_count', 10)

        # 步级握手：env→guard
        self.sub_step_start = self.create_subscription(UInt32, '/monitor_guard/step_start', self.on_step_start, 10)
        # 步级握手：guard→env
        self.pub_step_ack   = self.create_publisher(Int32MultiArray, '/monitor_guard/step_ack', 10)

        # 状态
        self.last_cmd_time = 0.0
        self.mute_until    = 0.0
        self.last_error_ts = -1e9
        self.violation_count = 0

        # step 窗口
        self.cur_step_id = None
        self.step_already_acked = False
        self.step_window_sec = 0.30  # 观测窗口（应 ≥ 单步动作时长）

        # 用定时器实现“到时自动 ack=0”
        self._timer_handle = None

        self.timer = self.create_timer(1.0 / REPUB_HZ, self.tick)
        self.get_logger().info('cmd_guard(minimal): pass-through + brake-on-error + step-ack (sim time)')

    # --- 工具 ---
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ack(self, violated: int):
        """发布与当前 step 绑定的 ack，一步只发一次。"""
        if self.cur_step_id is None or self.step_already_acked:
            return
        ack = Int32MultiArray()
        ack.data = [int(self.cur_step_id), int(violated)]
        self.pub_step_ack.publish(ack)
        self.step_already_acked = True

    # --- 回调 ---
    def on_cmd(self, msg: Twist):
        now = self._now()
        self.last_cmd_time = now

        # 静默期若收到“明确转向”则提前解除静默
        if now < self.mute_until:
            if abs(getattr(msg.angular, "z", 0.0)) >= TURN_WZ_EARLY_UNMUTE:
                self.mute_until = now
                self.pub_out.publish(msg)
            return

        # 正常直通
        self.pub_out.publish(msg)

    def on_verdict(self, msg):
        verdict = getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)
        v = (verdict or '').strip().lower()
        # 兼容 warning:2 的 "false" 与 warning:1 的 "currently_false"
        is_violation = (
            v == "false" or
            "currently_false" in v or
            "error" in v or
            "violation" in v
        )
        if not is_violation:
            return

        now = self._now()

        # 去抖：极密集 error 合并
        if (now - self.last_error_ts) < ERROR_COALESCE_S:
            # 但仍给当步 ack（防止 env 等不到）
            self._ack(1)
            return
        self.last_error_ts = now

        # 计数 + 发布
        self.violation_count += 1
        self.pub_violation_count.publish(UInt32(data=self.violation_count))
        self.get_logger().warn(f'monitor_verdict FALSE → brake & mute (violations={self.violation_count})')

        # 刹停 + 短静默（若当前不在静默）
        if now >= self.mute_until:
            self.pub_out.publish(Twist())
            self.mute_until = now + MUTE_SEC

        # 步级 ack：当前步判定违规
        self._ack(1)

    def on_step_start(self, msg: UInt32):
        """收到 env 的新一步开始，打开一个观测窗口。如果窗口内未见违规，自动回 ack=0。"""
        self.cur_step_id = int(msg.data)
        self.step_already_acked = False

        # 取消旧窗口
        if self._timer_handle is not None:
            try:
                self._timer_handle.cancel()
            except Exception:
                pass
            self._timer_handle = None

        def timeout_ack():
            # 到时仍未见违规 → ack=0
            self._ack(0)
            # 只触发一次
            if self._timer_handle is not None:
                try:
                    self._timer_handle.cancel()
                except Exception:
                    pass
                self._timer_handle = None

        # 用 ROS 定时器（仿真时钟）实现“窗口到时”
        self._timer_handle = self.create_timer(self.step_window_sec, timeout_ack)

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
