#!/usr/bin/env python3
import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TURN_WZ = 0.6        # rad/s, turning speed during override
OVERRIDE_SEC = 0.4   # seconds to override after a violation
TIMER_HZ = 20.0      # Hz for override publisher

# Try to import generated monitor message types; fall back to String if not present
try:
    from rosmonitoring_interfaces.msg import MonitorVerdict as VerdictMsg
except Exception:
    from std_msgs.msg import String as VerdictMsg  # fallback

try:
    from rosmonitoring_interfaces.msg import MonitorError as ErrorMsg
except Exception:
    from std_msgs.msg import String as ErrorMsg  # fallback


class CmdGuard(Node):
    """Pass-through by default; briefly overrides with a turning command on violations."""

    def __init__(self):
        super().__init__('cmd_guard')

        # Sub /cmd_vel_raw (filtered by monitor) and forward to /cmd_vel
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        # Listen to monitor verdicts and/or errors (whichever is available)
        self.sub_verdict = self.create_subscription(VerdictMsg, '/monitor_rl/monitor_verdict', self.on_verdict, 10)
        self.sub_error = self.create_subscription(ErrorMsg, '/monitor_rl/monitor_error', self.on_error, 10)

        self.pub_out = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.override_until = 0.0
        self.turn_dir = 1.0

        # Override loop timer
        self.timer = self.create_timer(1.0 / TIMER_HZ, self.tick)

        self.get_logger().info('cmd_guard is up: /cmd_vel_raw -> /cmd_vel, listening to /monitor_rl/*')

    # ----- Callbacks -----
    def on_cmd(self, msg: Twist):
        """Forward allowed commands unless currently overriding."""
        if time.time() < self.override_until:
            # Ignore incoming commands while overriding
            return
        self.pub_out.publish(msg)

    def _maybe_trigger_override(self, reason: str):
        now = time.time()
        self.override_until = now + OVERRIDE_SEC
        self.turn_dir = 1.0 if random.random() < 0.5 else -1.0
        self.get_logger().warn(f'Cmd override for {OVERRIDE_SEC:.2f}s due to {reason}, turn_dir={self.turn_dir:+.0f}')

    def on_verdict(self, msg):
        """Monitor verdict callback; trigger override on currently_false/false."""
        text = ''
        if hasattr(msg, 'verdict'):
            text = str(msg.verdict).lower()
        elif hasattr(msg, 'data'):
            text = str(msg.data).lower()
        else:
            text = str(msg).lower()

        if 'currently_false' in text or (text.strip() == 'false'):
            self._maybe_trigger_override('verdict')

    def on_error(self, msg):
        """Monitor error callback; conservative trigger if message looks like a violation."""
        text = ''
        if hasattr(msg, 'data'):
            text = str(msg.data).lower()
        else:
            text = str(msg).lower()
        if 'violation' in text or 'currently_false' in text or 'false' in text:
            self._maybe_trigger_override('error')

    def tick(self):
        """During override, publish a pure turning command to back out of danger."""
        now = time.time()
        if now < self.override_until:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.turn_dir * TURN_WZ
            self.pub_out.publish(twist)


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
