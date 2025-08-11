#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import numpy as np
import math
import time
from collections import deque


class GazeboEnvironment(Node):
    def __init__(self, node_name='rl_env'):
        super().__init__(node_name)

        # ------- ROS IO -------
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ------- State -------
        self.position = None
        self.orientation = None
        self.min_distance = float('inf')
        self.scan_state = np.array([3.5] * 10, dtype=float)
        self.goal = (3.0, 0.0)  # 终点
        self.prev_distance = None
        self.max_steps = 1000
        self.step_count = 0

        # 统计/辅助
        self.episode_crashes = 0
        self.episode_success = False
        self.last_reason = "continue"
        self.last_action = None
        self.prev_front = None
        self.prev_heading_err = None
        self.progress_win = deque(maxlen=20)  # 用于“卡住”检测

        # ------- Services -------
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        # ------- use_sim_time 安全设置 -------
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except Exception:
            try:
                self.declare_parameter("use_sim_time", True)
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                pass
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.get_logger().info(f"{node_name} node activated")

    # ===================== Callbacks =====================
    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.orientation = self.quaternion_to_yaw(orientation_q)

    def scan_callback(self, msg: LaserScan):
        scan = np.array(msg.ranges, dtype=float)
        rng_max = getattr(msg, "range_max", 3.5) or 3.5

        # 清洗：NaN/Inf/<=0 视为 range_max，避免当成“超近障碍”
        scan[~np.isfinite(scan)] = rng_max
        scan[scan <= 0.0] = rng_max
        scan = np.clip(scan, 0.0, rng_max)

        self.min_distance = float(np.min(scan))

        # 下采样为 10 束，均匀采样，确保包含正前方附近
        n = len(scan)
        if n > 0:
            idx = np.linspace(0, n - 1, 10, dtype=int)
            self.scan_state = scan[idx]

    # ===================== Utils =====================
    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_observation(self):
        if self.position is None or self.orientation is None:
            return None
        return np.concatenate([
            np.array([self.position.x, self.position.y, self.orientation], dtype=float),
            self.scan_state.astype(float)
        ])

    def compute_distance_to_goal(self):
        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        return math.hypot(dx, dy)

    # ===================== Reset =====================
    def reset(self):
        # 1) 重置仿真
        self.get_logger().info('Resetting the Gazebo world...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_simulation service...')
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('World reset completed.')

        # 2) 归位 TurtleBot（避免残余位姿导致一开局就贴墙）
        state = EntityState()
        state.name = "turtlebot3_burger"  # 如模型名不同请改
        state.pose.position.x = -3.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0  # yaw = 0
        req = SetEntityState.Request(); req.state = state
        if self.set_state_client.wait_for_service(timeout_sec=1.0):
            future2 = self.set_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future2)

        # 3) 强制清空，并等待新传感器数据
        self.position = None
        self.orientation = None
        self.min_distance = float('inf')
        self.prev_distance = None
        self.step_count = 0
        self.episode_crashes = 0
        self.episode_success = False
        self.last_reason = "continue"
        self.last_action = None
        self.progress_win.clear()

        self.get_logger().info('Waiting for initial sensor data...')
        while rclpy.ok() and (self.position is None or self.orientation is None or self.min_distance == float('inf')):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('Initial sensor data received.')

        # 起点到终点距离提示（可选）
        d0 = self.compute_distance_to_goal()
        if d0 < 0.3:
            self.get_logger().warn(f"Spawned too close to goal! Distance={d0:.2f}")

        return self.get_observation()


    # ===================== Step helpers =====================
    def base_step(self, linear, angular, duration_sim=0.25, control_hz=40):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)

        start = self.get_clock().now()
        period = 1.0 / control_hz

        while (self.get_clock().now() - start).nanoseconds * 1e-9 < duration_sim:
            self.vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.0)  # 只处理回调，不睡墙钟
            time.sleep(min(0.0005, period * 0.2))   # 防忙等
        # 停车
        self.vel_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.0)

    # ===================== Reward =====================
    def compute_reward(self, action, prev_action=None):
        # --- 进度与朝向 ---
        cur_dist = self.compute_distance_to_goal()
        if self.prev_distance is None:
            self.prev_distance = cur_dist
        progress = self.prev_distance - cur_dist
        self.prev_distance = cur_dist

        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        goal_angle = math.atan2(dy, dx)
        heading_err = (goal_angle - self.orientation + math.pi) % (2*math.pi) - math.pi
        d_heading = 0.0 if self.prev_heading_err is None else abs(heading_err - self.prev_heading_err)
        self.prev_heading_err = heading_err

        # --- 雷达三扇区 ---
        scan = np.asarray(self.scan_state, dtype=float)
        n = len(scan)
        if n >= 5:
            mid = n // 2
            front = float(np.min(scan[max(0, mid-1):min(n, mid+2)]))  # 中间3束
            left  = float(np.min(scan[mid:])) if mid < n else float('inf')
            right = float(np.min(scan[:mid])) if mid > 0 else float('inf')
        else:
            front = float(self.min_distance); left = right = float(self.min_distance)

        # --- 分段阈值 ---
        SAFE_HEADING   = 0.80  # 仅在足够通畅时才按朝向给奖励
        FRONT_SOFT     = 0.75  # 前方拥挤软惩罚阈值
        FRONT_HARD     = 0.55  # 前方很近硬惩罚阈值
        SIDE_SAFE      = 0.35  # 直行时靠边惩罚
        EARLY_GRACE    = 3

        # --- 基础奖励：进度为主 ---
        reward = 8.0 * progress

        # 只在前方通畅时才给朝向奖励；不通畅时角度不再驱动“直冲”
        if front >= SAFE_HEADING:
            reward += 0.25 * math.cos(heading_err)

        # 直行时按“前方距离变化”给反馈（抑制直行抖头拿高分）
        if action == 0:
            if self.prev_front is not None:
                df = front - self.prev_front
                reward += 0.4 * max(0.0, df)      # 前方更通畅 → 小奖
                reward -= 0.6 * max(0.0, -df)     # 前方更拥挤 → 扣分
            # 靠边直行扣分
            side_min = min(left, right)
            if side_min < SIDE_SAFE:
                reward -= 1.5 * (SIDE_SAFE - side_min)

        # 前方近障仍直行 → 强惩罚
        if action == 0 and front < FRONT_SOFT:
            k = 5.0 if front < FRONT_HARD else 3.0
            reward -= k * (FRONT_SOFT - front)

        # 转向更空的一侧 → 小奖（仅正向差值）
        if action == 1:
            reward += 0.25 * max(0.0, (left - right))
        elif action == 2:
            reward += 0.25 * max(0.0, (right - left))

        # 防抖1：左右切换就罚
        if prev_action in (1,2) and action in (1,2) and prev_action != action:
            reward -= 0.12

        # 防抖2：角度误差剧烈变化（“摇头”）在前方通畅时罚
        if front >= SAFE_HEADING:
            reward -= 0.10 * d_heading  # 单位是弧度差

        # 全局安全温和惩罚 + 轻时间惩罚
        if self.min_distance < 0.50:
            reward -= 2.0 * (0.50 - self.min_distance)
        reward -= 0.01

        # === “卡住”检测：把 progress 滑窗起来，进展过小就早停 ===
        self.progress_win.append(progress)
        if (len(self.progress_win) == self.progress_win.maxlen) and (self.step_count > 40):
            if sum(self.progress_win) < 0.02:   # 近 20 步总进展 < 2cm（可按你的环境单位适当调）
                reward -= 5.0
                done = True
                self.last_reason = "stuck"
                self.get_logger().info("early stop: stuck")
                # 记住前方距离并返回
                self.prev_front = front
                return reward, done

        # --- 终止判定（带前几步宽限） ---
        if cur_dist < 0.30:
            reward += 40.0; done = True; self.last_reason="reach"; self.episode_success=True; self.get_logger().info("reach the end")
        elif (self.step_count > EARLY_GRACE) and (self.min_distance < 0.20):
            reward -= 30.0; done = True; self.last_reason="crush"; self.episode_crashes += 1; self.get_logger().info("crush")
        elif self.step_count >= self.max_steps:
            reward -= 10.0; done = True; self.last_reason="overtime"; self.get_logger().info("overtime")
        else:
            done = False; self.last_reason="continue"

        # 记住前方距离用于“前方变好/变差”反馈
        self.prev_front = front
        return reward, done


    def is_violation(self):
        return self.min_distance < 0.2


# ===================== Envs =====================
class NoMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        """Discrete action: 0=forward, 1=left, 2=right"""
        if action == 0:
            self.base_step(0.25, 0.0, duration_sim=0.20, control_hz=40)
        elif action == 1:
            self.base_step(0.0, 0.55, duration_sim=0.18, control_hz=40)
        elif action == 2:
            self.base_step(0.0, -0.55, duration_sim=0.18, control_hz=40)

        self.step_count += 1
        reward, done = self.compute_reward(action)
        obs = self.get_observation()

        info = {
            "reason": self.last_reason,
            "distance_to_goal": float(self.compute_distance_to_goal()),
            "min_laser": float(self.min_distance),
            "step_count": int(self.step_count),
            "violation": bool(self.is_violation()),
            "success": bool(self.episode_success),
            "crashes_in_episode": int(self.episode_crashes),
        }
        return obs, reward, done, info


class PassiveMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        if self.is_violation():
            self.get_logger().warn("Passive Monitor: Violation detected!")
        return super().step(action)


class ActiveMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        # 前方太近时禁止前进，随机转向（保底不首撞）
        front = float(self.scan_state[len(self.scan_state) // 2])
        if action == 0 and front < 0.35:
            self.get_logger().warn("Active Monitor: block forward, turn!")
            action = 1 if np.random.rand() < 0.5 else 2
        return super().step(action)
