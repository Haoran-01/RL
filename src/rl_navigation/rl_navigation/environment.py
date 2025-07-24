#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import numpy as np
import math
import time

class GazeboEnvironment(Node):
    def __init__(self, node_name='rl_env'):
        super().__init__(node_name)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.position = None
        self.orientation = None
        self.min_distance = float('inf')
        self.goal = (0.0, 0.0)  # 终点坐标
        self.prev_distance = None
        self.max_steps = 500
        self.step_count = 0

        # Gazebo service client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        self.get_logger().info(f"{node_name} node activated")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.orientation = self.quaternion_to_yaw(orientation_q)

    def scan_callback(self, msg):
        scan = np.array(msg.ranges)
        scan = np.clip(scan, 0.0, 3.5)  # Clip scan range
        self.min_distance = np.min(scan)
        self.scan_state = scan[::len(scan)//10]  # Downsample to 10 beams

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_observation(self):
        if self.position is None or self.orientation is None:
            return None
        return np.concatenate([
            np.array([self.position.x, self.position.y, self.orientation]),
            self.scan_state
        ])

    def compute_distance_to_goal(self):
        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        return math.hypot(dx, dy)

    def reset(self):
        # Reset Gazebo world
        self.get_logger().info('Resetting the Gazebo world...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_world service...')
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('World reset completed.')

        # Wait for odom and scan to update
        self.get_logger().info('Waiting for initial sensor data...')
        while rclpy.ok() and (self.position is None or self.orientation is None or self.min_distance == float('inf')):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Initial sensor data received.')

        self.prev_distance = None
        self.step_count = 0
        return self.get_observation()


    def base_step(self, linear, angular, duration=0.7):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)
        rclpy.spin_once(self, timeout_sec=duration)
        self.vel_pub.publish(Twist())  # Stop after action

    # def compute_reward(self):
    #     current_distance = self.compute_distance_to_goal()
      
    #     # self.get_logger().info(f"Distance to goal: {current_distance:.2f}, Min laser: {self.min_distance:.2f}")


    #     if self.prev_distance is None:
    #         self.prev_distance = current_distance
    #     distance_diff = self.prev_distance - current_distance
    #     reward = distance_diff * 10.0 # Scale

    #     # 到达终点
    #     if current_distance < 0.3:
    #         reward += 100.0
    #         done = True
    #         self.get_logger().info(f"reach the end")
    #     # 碰撞
    #     elif self.min_distance < 0.2:
    #         reward -= 100.0
    #         done = True
    #         self.get_logger().info(f"crush")
    #     # 超时
    #     elif self.step_count >= self.max_steps:
    #         reward -= 50.0
    #         done = True
    #         self.get_logger().info(f"overtime")
    #     else:
    #         done = False

    #     self.prev_distance = current_distance
    #     return reward, done


    def compute_reward(self, action):
        current_distance = self.compute_distance_to_goal()

        if self.prev_distance is None:
            self.prev_distance = current_distance
        distance_diff = self.prev_distance - current_distance

        # # 朝向目标方向奖励
        # dx = self.goal[0] - self.position.x
        # dy = self.goal[1] - self.position.y
        # heading_to_goal = math.atan2(dy, dx)
        # angle_diff = abs(self.orientation - heading_to_goal)
        # angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
        # angle_reward = (math.pi - angle_diff) / math.pi

        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        goal_angle = math.atan2(dy, dx)
        heading_error = goal_angle - self.orientation
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  # wrap to [-π, π]

        angle_reward = -abs(heading_error)
    

        # 基础奖励：接近目标 + 朝向目标
        reward = distance_diff * 10.0 + angle_reward

        # 靠近障碍物惩罚
        if self.min_distance < 0.5:
            reward -= (0.5 - self.min_distance) * 2

        # 原地旋转惩罚
        if action in [1, 2]:
            reward -= 0.02

        # 时间惩罚
        reward -= 0.05

        # 终止状态奖励
        if current_distance < 0.3:
            reward += 30.0
            done = True
            self.get_logger().info(f"reach the end")
        elif self.min_distance < 0.2:
            reward -= 30.0
            done = True
            self.get_logger().info(f"crush")
        elif self.step_count >= self.max_steps:
            reward -= 10.0
            done = True
            self.get_logger().info(f"overtime")
        else:
            done = False

        self.prev_distance = current_distance
        return reward, done


    def is_violation(self):
        return self.min_distance < 0.2


class NoMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        """Discrete action: 0=forward, 1=left, 2=right"""
        if action == 0:
            self.base_step(0.25, 0.0)
        elif action == 1:
            self.base_step(0.0, 0.5)
        elif action == 2:
            self.base_step(0.0, -0.5)

        self.step_count += 1
        reward, done = self.compute_reward(action)
        obs = self.get_observation()
        return obs, reward, done, {}


class PassiveMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        if self.is_violation():
            self.get_logger().warn("Passive Monitor: Violation detected!")
        return super().step(action)


class ActiveMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        if self.is_violation() and action == 0:  # Trying to move forward into obstacle
            self.get_logger().warn("Active Monitor: Stopping action due to violation!")
            action = 1  # Replace with turn left
        return super().step(action)
