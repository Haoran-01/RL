# #!/usr/bin/env python3
# import time
# import random
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# # -------- Parameters --------
# TURN_WZ       = 0.45     # 接管时角速度
# OVERRIDE_SEC  = 0.25     # 接管持续时间
# TIMER_HZ      = 30.0     # 定时器频率（同时用于重发/看门狗）
# STALE_SEC     = 0.80     # 超过这段时间没新指令 -> 认为“陈旧”，改发零速
# REPUB_HZ      = 12.0     # 在陈旧前，按这个低频重发最近一条正常指令
# REARM_SEC     = 1.5      # 接管的“冷却窗”，避免连续触发

# # monitor 消息类型兜底
# try:
#     from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
# except Exception:
#     from std_msgs.msg import String as VerdictMsg

# try:
#     from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
# except Exception:
#     from std_msgs.msg import String as ErrorMsg


# class CmdGuard(Node):
#     """Pass-through by default; short override on violations; heartbeat/hold for sparse cmds."""

#     def __init__(self):
#         super().__init__('cmd_guard')

#         # IO
#         self.sub_cmd     = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
#         self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
#         self.sub_error   = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)
#         self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)

#         # State
#         self.override_until  = 0.0
#         self.turn_dir        = 1.0
#         self.was_overriding  = False

#         self.last_cmd_time   = 0.0         # 最近一次收到 /cmd_vel_raw 的时间
#         self.last_cmd_msg    = None        # 最近一次的“正常指令”
#         self.last_repub_time = 0.0         # 最近一次重发 normal cmd 的时间
#         self.last_arm_time   = 0.0         # 最近一次因 error 接管的时间

#         # Timer
#         self.timer = self.create_timer(1.0 / max(TIMER_HZ, REPUB_HZ), self.tick)

#         self.get_logger().info('cmd_guard up: /cmd_vel_raw -> /cmd_vel, listening /monitor_rl/*')

#     # ---------- Callbacks ----------
#     def on_cmd(self, msg: Twist):
#         """正常透传：记录&转发最近的正常指令；覆盖期丢弃。"""
#         now = time.time()
#         if now < self.override_until:
#             return
#         self.last_cmd_time = now
#         self.last_cmd_msg  = msg
#         self.pub_out.publish(msg)

#     def on_verdict(self, msg):
#         """verdict 仅记录（避免每个 false 都接管）"""
#         text = (getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)).lower().strip()
#         self.get_logger().info(f'verdict={text}')

#     def on_error(self, msg):
#         """仅在最终 false（warning=2）时触发，且有冷却窗"""
#         now = time.time()
#         if (now - self.last_arm_time) < REARM_SEC:
#             return
#         self.last_arm_time   = now
#         self.override_until  = now + OVERRIDE_SEC
#         self.turn_dir        = 1.0 if random.random() < 0.5 else -1.0
#         self.get_logger().warn(f'Cmd override for {OVERRIDE_SEC:.2f}s due to oracle_false, dir={self.turn_dir:+.0f}')

#     # ---------- Timer ----------
#     def tick(self):
#         now = time.time()

#         # 覆盖期：纯转向
#         if now < self.override_until:
#             self.was_overriding = True
#             tw = Twist()
#             tw.linear.x  = 0.0
#             tw.angular.z = self.turn_dir * TURN_WZ
#             self.pub_out.publish(tw)
#             return

#         # 覆盖刚结束：补一帧零速，清一下惯性
#         if self.was_overriding:
#             self.was_overriding = False
#             self.pub_out.publish(Twist())

#         # —— 稀疏指令的“保持/看门狗”策略 ——
#         dt = now - self.last_cmd_time

#         if dt <= STALE_SEC:
#             # 还没到“陈旧”阈值：以低频重发最近一条正常指令，让动作持续
#             if self.last_cmd_msg is not None and (now - self.last_repub_time) >= (1.0 / REPUB_HZ):
#                 self.last_repub_time = now
#                 self.pub_out.publish(self.last_cmd_msg)
#         else:
#             # 已经“陈旧”了：发布零速，避免底盘继续执行上一条（尤其是转向）
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
# -*- coding: utf-8 -*-
"""
cmd_guard.py — 在不修改 environment 奖励函数的前提下，通过指令层优化抑制“绕圈”：
1) 对“转向指令”的续命时间上限（MAX_TURN_HOLD），避免被低频重发扩展成长期旋转；
2) 反打转看门狗（anti‑spin）：连续满足“|wz|大 & vx小”达到阈值则急停，并进入短暂静默期；
3) （可选）订阅 /scan_min（Float32），仅在确实“近墙”时允许短续命转向。
"""

import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ====================== 参数区（可按需调整） ======================

# 监控触发时（oracle false）短时接管的旋转
TURN_WZ       = 0.45     # rad/s
OVERRIDE_SEC  = 0.25     # s
REARM_SEC     = 1.5      # s，接管冷却窗，避免高频触发

# 定时器与“心跳重发”节流
TIMER_HZ      = 30.0     # tick 频率（同时决定看门狗积分步长）
REPUB_HZ      = 12.0     # 在“未陈旧”窗口里重发最近正常指令的低频
STALE_SEC     = 0.80     # s，超过该时间没新指令视作“陈旧”→发零速

# 反打转 & 转向续命限制
MAX_TURN_HOLD   = 0.25   # s，最近一条“转向指令”可被续命的最长时间
SPIN_WZ_THR     = 0.40   # rad/s，判定“在打转”的角速度阈值
SPIN_VX_MAX     = 0.05   # m/s，打转时线速度需很小
SPIN_SEC        = 0.50   # s，连续判定为“打转”的累计时长阈值
MUTE_AFTER_SPIN = 0.60   # s，触发反打转后静默期：不重发旧指令

# 是否结合 /scan_min（Float32）决定是否允许转向续命
USE_SCAN_MIN         = True
TURN_HOLD_NEEDS_NEAR = 0.60  # m，仅当 last_scan < 该值时才允许“续命转向”


# ====================== 兼容 rosmonitoring 消息类型 ======================

try:
    from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
except Exception:
    from std_msgs.msg import String as VerdictMsg

try:
    from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
except Exception:
    from std_msgs.msg import String as ErrorMsg


# ====================== 节点实现 ======================

class CmdGuard(Node):
    """
    - 默认透传 /cmd_vel_raw -> /cmd_vel；
    - oracle 报错（最终 false）时短时接管：原地小角速旋转 OVERRIDE_SEC；
    - 稀疏指令时：对“直行”可心跳续命；“转向”仅在 MAX_TURN_HOLD 内短续命；
    - 若检测到连续“在打转”，触发 anti‑spin：急停并静默 MUTE_AFTER_SPIN。
    """

    def __init__(self):
        super().__init__('cmd_guard')

        # 订阅/发布
        self.sub_cmd     = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
        self.sub_error   = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)
        self.pub_out     = self.create_publisher(Twist, '/cmd_vel', 10)

        # （可选）订阅 /scan_min
        self.last_scan = None
        if USE_SCAN_MIN:
            try:
                from std_msgs.msg import Float32
                self.create_subscription(Float32, '/scan_min', self.on_scan_min, 10)
                self.get_logger().info('cmd_guard: using /scan_min to gate turn-hold.')
            except Exception:
                self.get_logger().warn('cmd_guard: /scan_min unavailable; proceeding without it.')

        # 接管状态
        self.override_until  = 0.0
        self.turn_dir        = 1.0
        self.was_overriding  = False
        self.last_arm_time   = 0.0

        # 最近指令缓存与心跳重发
        self.last_cmd_time   = 0.0
        self.last_cmd_msg    = None
        self.last_repub_time = 0.0

        # 转向续命与打转看门狗
        self.last_cmd_is_turn = False
        self.turn_hold_start  = 0.0
        self.spin_accum       = 0.0
        self.mute_until       = 0.0

        # 定时器
        self.timer = self.create_timer(1.0 / max(1.0, max(TIMER_HZ, REPUB_HZ)), self.tick)

        self.get_logger().info('cmd_guard up: /cmd_vel_raw -> /cmd_vel, listening /monitor_rl/*')

    # -------------------- 订阅回调 --------------------

    def on_scan_min(self, msg):
        try:
            self.last_scan = float(getattr(msg, 'data', None))
        except Exception:
            pass

    def on_cmd(self, msg: Twist):
        """正常透传：记录并转发最近的正常指令；在覆盖期丢弃上游指令。"""
        now = time.time()
        if now < self.override_until:
            return

        # 是否为“强转向型”指令（角速度大、线速度小）
        is_turn = (abs(msg.angular.z) >= SPIN_WZ_THR and abs(msg.linear.x) <= SPIN_VX_MAX)
        if is_turn and not self.last_cmd_is_turn:
            self.turn_hold_start = now
        self.last_cmd_is_turn = is_turn

        self.last_cmd_time = now
        self.last_cmd_msg  = msg
        self.pub_out.publish(msg)

    def on_verdict(self, msg):
        text = (getattr(msg, 'verdict', '') or getattr(msg, 'data', '') or str(msg)).lower().strip()
        if text:
            self.get_logger().info(f'verdict={text}')

    def on_error(self, msg):
        """仅在最终 false（warning=2）时触发短时接管，并遵守冷却窗。"""
        now = time.time()
        if (now - self.last_arm_time) < REARM_SEC:
            return
        self.last_arm_time  = now
        self.override_until = now + OVERRIDE_SEC
        self.turn_dir       = 1.0 if random.random() < 0.5 else -1.0
        self.get_logger().warn(f'Cmd override for {OVERRIDE_SEC:.2f}s due to oracle_false, dir={self.turn_dir:+.0f}')

    # -------------------- 定时逻辑 --------------------

    def tick(self):
        now = time.time()

        # 覆盖期：纯转向接管
        if now < self.override_until:
            self.was_overriding = True
            tw = Twist()
            tw.linear.x  = 0.0
            tw.angular.z = self.turn_dir * TURN_WZ
            self.pub_out.publish(tw)
            return

        # 覆盖刚结束：补一帧零速，清除惯性
        if self.was_overriding:
            self.was_overriding = False
            self.pub_out.publish(Twist())

        # 反打转（anti‑spin）累计：最近缓存为“强转向”且线速度小
        turning = False
        if self.last_cmd_msg is not None:
            lc = self.last_cmd_msg
            turning = (abs(lc.angular.z) >= SPIN_WZ_THR and abs(lc.linear.x) <= SPIN_VX_MAX)

        if turning:
            # 以定时器步长积分
            self.spin_accum = min(SPIN_SEC + 1.0, self.spin_accum + 1.0 / max(1.0, TIMER_HZ))
        else:
            # 衰减更快一些，避免偶发抖动误触
            self.spin_accum = max(0.0, self.spin_accum - 2.0 / max(1.0, TIMER_HZ))

        # 触发 anti‑spin：急停 + 清缓存 + 静默
        if self.spin_accum >= SPIN_SEC:
            self.get_logger().warn('anti-spin: stop & clear last command')
            self.pub_out.publish(Twist())
            self.last_cmd_msg     = None
            self.last_cmd_is_turn = False
            self.spin_accum       = 0.0
            self.mute_until       = now + MUTE_AFTER_SPIN
            return

        # 在静默期内：不重发任何旧指令
        if now < self.mute_until:
            return

        # —— 稀疏指令的“保持/看门狗”策略 ——
        dt = now - self.last_cmd_time

        if dt <= STALE_SEC and self.last_cmd_msg is not None:
            # 还未“陈旧”：按低频重发最近正常指令
            if (now - self.last_repub_time) >= (1.0 / max(1.0, REPUB_HZ)):
                if self.last_cmd_is_turn:
                    # 对“转向指令”：仅短续命，且（可选）仅在近墙时才允许
                    hold_ok = (now - self.turn_hold_start) <= MAX_TURN_HOLD
                    near_ok = True
                    if USE_SCAN_MIN:
                        near_ok = (self.last_scan is not None and self.last_scan < TURN_HOLD_NEEDS_NEAR)

                    if hold_ok and near_ok:
                        self.last_repub_time = now
                        self.pub_out.publish(self.last_cmd_msg)
                    else:
                        # 超过续命时间或不需要转向时：以零速代替，等待上层新决策
                        self.pub_out.publish(Twist())
                else:
                    # 直行可续命：维持推进
                    self.last_repub_time = now
                    self.pub_out.publish(self.last_cmd_msg)
        else:
            # 已“陈旧”：发零速，避免底盘延续旧动作（尤其是转向）
            self.pub_out.publish(Twist())


# ====================== 入口 ======================

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
