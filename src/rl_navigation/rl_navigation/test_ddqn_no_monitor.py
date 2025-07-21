import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import torch
import numpy as np
from ddqn_agent import DDQNAgent  # 你的DDQNAgent类
import time

class NoMonitorTester(Node):
    def __init__(self):
        super().__init__('no_monitor_tester')
        self.agent = DDQNAgent(state_dim=10, action_dim=3)
        self.agent.q_network.load_state_dict(torch.load('ddqn_model.pth'))  # 加载已训练模型
        self.state = None
        self.done = False

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    def scan_callback(self, msg):
        scan = np.array(msg.ranges)
        scan = np.clip(scan, 0, 3.5)
        downsampled = scan[::len(scan)//10]
        self.state = downsampled

    def run_episode(self):
        self.reset_robot()
        total_reward = 0
        steps = 0
        while rclpy.ok() and not self.done:
            if self.state is None:
                continue
            action = self.agent.act(self.state)
            self.publish_cmd(action)
            total_reward += 0  # 这里你可以根据距离终点给 reward
            steps += 1
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"Episode finished: reward={total_reward}, steps={steps}")

    def publish_cmd(self, action):
        cmd = Twist()
        if action == 0:
            cmd.linear.x = 0.2
        elif action == 1:
            cmd.angular.z = 0.5
        elif action == 2:
            cmd.angular.z = -0.5
        self.cmd_pub.publish(cmd)

    def reset_robot(self):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'turtlebot3'
        req.state.pose.position.x = -3.0
        req.state.pose.position.y = 0.0
        req.state.pose.orientation.w = 1.0
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    tester = NoMonitorTester()
    for _ in range(10):  # 测试10轮
        tester.run_episode()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
