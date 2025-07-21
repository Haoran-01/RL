#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
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
        self.get_logger().info(f"{node_name} node activate")
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')


    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.orientation = self.quaternion_to_yaw(orientation_q)

    def scan_callback(self, msg):
        self.min_distance = min(msg.ranges)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_state(self):
        if self.position and self.orientation:
            return (self.position.x, self.position.y, self.orientation)
        return None

    def reset(self):
        # Reset the Gazebo simulation
        self.get_logger().info('Resetting the Gazebo simulation...')
        client = self.create_client(Empty, '/reset_simulation')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_simulation service...')
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Set the robot to the fixed start position
        self.get_logger().info('Setting the robot to the fixed start position...')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        state = EntityState()
        state.name = 'car_model'

        # Fixed start position and orientation
        state.pose.position.x = -4.29761
        state.pose.position.y = 4.17789
        state.pose.position.z = 0.01
        yaw = 0.0

        # Convert yaw to quaternion
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)

        req_state = SetEntityState.Request()
        req_state.state = state
        future = self.set_state_client.call_async(req_state)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(
            f"Robot reset to fixed start position: (x={state.pose.position.x:.2f}, y={state.pose.position.y:.2f}, yaw={yaw:.2f} rad)"
        )

    def is_violation(self):
        return self.min_distance < 0.2


class NoMonitoringEnv(GazeboEnvironment):
    def step(self, linear=0.2, angular=0.0, duration=1.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)
        time.sleep(duration)
        self.vel_pub.publish(Twist())


class PassiveMonitoringEnv(GazeboEnvironment):
    def step(self, linear=0.2, angular=0.0, duration=1.0):
        if self.min_distance < 0.2 and linear > 0:
            self.get_logger().warn("warning")
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)
        time.sleep(duration)
        self.vel_pub.publish(Twist())


class ActiveMonitoringEnv(GazeboEnvironment):
    def step(self, linear=0.2, angular=0.0, duration=1.0):
        cmd = Twist()
        if self.min_distance < 0.2 and linear > 0:
            self.get_logger().warn("stop")
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)
        time.sleep(duration)
        self.vel_pub.publish(Twist())
