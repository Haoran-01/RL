# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from geometry_msgs.msg import Twist
# import time  # 保留，不再用于计时（仅备用）

# # ===== Monitor tunables =====
# MUTE_SEC  = 0.20   # [CHANGED] 由 0.40 → 0.20，更贴合每步动作 ~0.18–0.20s
# STALE_SEC = 0.80
# REPUB_HZ  = 10.0

# TURN_WZ_EARLY_UNMUTE = 0.35  # [NEW] “明确转向”阈值；静默期若收到 |wz|≥该值的转向，提前解锁
# ERROR_COALESCE_S     = 0.10  # [NEW] 合并极密集的 error（防“刹停风暴”）

# try:
#     from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
#     from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
# except Exception:
#     from std_msgs.msg import String as ErrorMsg
#     from std_msgs.msg import String as VerdictMsg


# class CmdGuard(Node):
#     """平时直通；监控报错时仅刹停并短静默，不注入转向。使用仿真时钟。"""
#     def __init__(self):
#         super().__init__('cmd_guard_minimal')

#         # --- 使用仿真时钟 --- #
#         # [NEW] 开启 use_sim_time，所有计时与 Gazebo /clock 对齐
#         try:
#             if not self.has_parameter("use_sim_time"):
#                 self.declare_parameter("use_sim_time", True)
#         except Exception:
#             pass
#         self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])  # [NEW]

#         # 订阅/发布
#         self.sub_cmd     = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
#         self.sub_error   = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)
#         self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
#         self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)

#         # 状态
#         self.last_cmd_time = 0.0
#         self.mute_until    = 0.0
#         self.last_error_ts = 0.0

#         # [CHANGED] 用 ROS 时钟创建定时器（随仿真时钟走）
#         self.timer = self.create_timer(1.0 / REPUB_HZ, self.tick)

#         self.get_logger().info('cmd_guard(minimal): pass-through + brake-on-error (sim time)')

#     def _now(self) -> float:
#         """仿真时钟（秒）。"""
#         return self.get_clock().now().nanoseconds * 1e-9  # [NEW]

#     def on_cmd(self, msg: Twist):
#         now = self._now()
#         self.last_cmd_time = now

#         # [NEW] 静默期若收到“明确转向”命令则提前解除静默并转发
#         if now < self.mute_until:
#             if abs(getattr(msg.angular, "z", 0.0)) >= TURN_WZ_EARLY_UNMUTE:
#                 self.mute_until = now
#                 self.pub_out.publish(msg)
#             return

#         # 正常直通
#         self.pub_out.publish(msg)

#     def on_error(self, _msg):
#         now = self._now()

#         # [NEW] 合并极密集的 error，避免连续多次触发导致长时间卡死
#         if (now - self.last_error_ts) < ERROR_COALESCE_S:
#             return
#         self.last_error_ts = now

#         self.get_logger().warn('monitor_error → BRAKE & mute')
#         self.pub_out.publish(Twist())            # 立刻刹停
#         self.mute_until = now + MUTE_SEC         # [CHANGED] 更短静默

#     def on_verdict(self, msg):
#         verdict = getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)
#         if verdict:
#             self.get_logger().info(f'verdict={verdict}')

#     def tick(self):
#         # 定期兜底：太久没新命令就发零速
#         if self._now() - self.last_cmd_time > STALE_SEC:
#             self.pub_out.publish(Twist())


# def main():
#     rclpy.init()
#     node = CmdGuard()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
# cmd_guard_passthrough.py — EXACT pass-through: /cmd_vel_raw -> /cmd_vel
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdGuardPassthrough(Node):
    def __init__(self):
        super().__init__('cmd_guard_passthrough')
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.pub_out = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('cmd_guard_passthrough: direct relay (no timers, no zeros, no errors)')

    def on_cmd(self, msg: Twist):
        self.pub_out.publish(msg)

def main():
    rclpy.init()
    node = CmdGuardPassthrough()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
