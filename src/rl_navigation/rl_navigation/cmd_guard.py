#!/usr/bin/env python3
import time, rclpy, random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

TURN_WZ = 0.6      # rad/s
OVERRIDE_SEC = 0.4 # seconds

class CmdGuard(Node):
    def __init__(self):
        super().__init__('cmd_guard')
        # Allowed (filtered) commands come from /cmd_vel_raw after monitor filter
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd, 10)
        self.sub_err = self.create_subscription(String, '/monitor_rl/monitor_error', self.on_violation, 10)
        self.pub_out = self.create_publisher(Twist, '/cmd_vel', 10)
        self.override_until = 0.0
        self.turn_dir = 1.0
        self.timer = self.create_timer(1/30.0, self.tick)

    def on_cmd(self, msg: Twist):
        # Forward allowed commands unless we are overriding
        if time.time() < self.override_until:
            return
        self.pub_out.publish(msg)

    def on_violation(self, _msg: String):
        # Any violation -> issue a short turning override
        self.override_until = time.time() + OVERRIDE_SEC
        self.turn_dir = 1.0 if random.random() < 0.5 else -1.0

    def tick(self):
        # During override, repeatedly send turn command (no forward)
        if time.time() < self.override_until:
            m = Twist(); m.linear.x = 0.0; m.angular.z = self.turn_dir * TURN_WZ
            self.pub_out.publish(m)

def main():
    rclpy.init(); n=CmdGuard(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
