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
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.position = None
        self.orientation = None
        
        self.progress_win = deque(maxlen=20)    # 用于检测“卡住”（约0.2~0.5s窗口，随step频率而定）
        self.last_action = None   # 0=前进,1=左,2=右
        self.min_distance = float('inf')
        self.goal = (3.0, 0.0)  # 终点坐标
        self.prev_distance = None
        self.max_steps = 1000
        self.step_count = 0

        # ... GazeboEnvironment.__init__(...)
        # ⭐ 安全声明 + 赋值（避免重复声明异常）
        try:
            if not self.has_parameter("use_sim_time"):
                self.declare_parameter("use_sim_time", True)
        except Exception:
            # 某些发行版没有 has_parameter() 或其它原因，直接忽略重复声明异常
            try:
                self.declare_parameter("use_sim_time", True)
            except rclpy.exceptions.ParameterAlreadyDeclaredException:
                pass

        # 不管是否是新声明的，都把值设为 True
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # Gazebo service client
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        self.get_logger().info(f"{node_name} node activated")

        self.episode_crashes = 0
        self.episode_success = False
        self.last_reason = "continue"  # reach / crush / overtime / continue


        


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

    # def reset(self):
    #     # Reset Gazebo world
    #     self.get_logger().info('Resetting the Gazebo world...')
    #     while not self.reset_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Waiting for /reset_world service...')
    #     req = Empty.Request()
    #     future = self.reset_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     self.get_logger().info('World reset completed.')

    #     # Wait for odom and scan to update
    #     self.get_logger().info('Waiting for initial sensor data...')
    #     while rclpy.ok() and (self.position is None or self.orientation is None or self.min_distance == float('inf')):
    #         rclpy.spin_once(self, timeout_sec=0.1)
    #     self.get_logger().info('Initial sensor data received.')


    #     self.prev_distance = None
    #     self.step_count = 0
    #     return self.get_observation()

    def reset(self):
        # Reset Gazebo world
        self.get_logger().info('Resetting the Gazebo world...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_world service...')
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('World reset completed.')

        # 🟡 强制清空所有状态（非常关键）
        self.position = None
        self.orientation = None
        self.min_distance = float('inf')   # ✅ 没有这行可能导致刚开始就触发碰撞done
        self.prev_distance = None
        self.step_count = 0

        # 🟢 等待新的传感器数据
        self.get_logger().info('Waiting for initial sensor data...')
        while rclpy.ok() and (self.position is None or self.orientation is None or self.min_distance == float('inf')):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Initial sensor data received.')

        # 🔴 [可选] 检查是否初始就已到达终点，避免done立即触发
        current_distance = self.compute_distance_to_goal()
        if current_distance < 0.3:
            self.get_logger().warn(f"Spawned too close to goal! Distance={current_distance:.2f}")


        self.episode_crashes = 0
        self.episode_success = False
        self.last_reason = "continue"  # reach / crush / overtime / continue


        return self.get_observation()



    # def base_step(self, linear, angular, duration=1.0):
    #     cmd = Twist()
    #     cmd.linear.x = linear
    #     cmd.angular.z = angular
    #     self.vel_pub.publish(cmd)
    #     rclpy.spin_once(self, timeout_sec=duration)
    #     self.vel_pub.publish(Twist())  # Stop after action


    def base_step(self, linear, angular, duration_sim=0.5, control_hz=40):
        """
        duration_sim: 本次动作持续的【仿真时间】秒（不是墙钟）
        control_hz:   控制频率（每秒发布多少次速度指令）
        """
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        # 以仿真时间为基准推进
        start = self.get_clock().now()
        period = 1.0 / control_hz

        while (self.get_clock().now() - start).nanoseconds * 1e-9 < duration_sim:
            self.vel_pub.publish(cmd)
            # 处理消息，不等待墙钟（0.0），Gazebo RTF>1 时会随之加速
            rclpy.spin_once(self, timeout_sec=0.0)
            # 用一个很短的墙钟小睡，避免CPU 100%忙等；值越小越快
            time.sleep(min(0.0005, period * 0.2))

        # 停车，并处理最后一批回调
        self.vel_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.0)


    # def compute_reward(self, action, prev_action=None):
    #     """
    #     奖励塑形要点：
    #     - 弱化“朝向目标”奖励，避免死盯直线
    #     - 利用激光的前/左/右扇区做避障引导
    #     * 前方很近仍前进 -> 大惩罚
    #     * 向更空的一侧转弯 -> 小奖励
    #     - 降低原地旋转惩罚，鼓励探索
    #     """
    #     # --- 距离与朝向 ---
    #     current_distance = self.compute_distance_to_goal()
    #     if self.prev_distance is None:
    #         self.prev_distance = current_distance
    #     distance_diff = self.prev_distance - current_distance

    #     dx = self.goal[0] - self.position.x
    #     dy = self.goal[1] - self.position.y
    #     goal_angle = math.atan2(dy, dx)
    #     heading_error = goal_angle - self.orientation
    #     heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  # wrap [-pi, pi]

    #     # 弱化“朝向目标”项（避免直线硬顶）
    #     angle_reward = -0.3 * abs(heading_error)

    #     # 基础奖励：靠近目标 + 朝向目标
    #     reward = distance_diff * 10.0 + angle_reward

    #     # --- 利用激光扇区做局部避障引导 ---
    #     scan = np.asarray(self.scan_state, dtype=float) if hasattr(self, "scan_state") else np.array([self.min_distance])
    #     n = len(scan)
    #     if n >= 3:
    #         mid = n // 2
    #         # 前方取中间2个beam的最小值，更稳健
    #         front = np.min(scan[max(0, mid-1):min(n, mid+1)])
    #         left  = np.min(scan[mid:]) if mid < n else float('inf')
    #         right = np.min(scan[:mid]) if mid > 0 else float('inf')
    #     else:
    #         front = self.min_distance
    #         left, right = self.min_distance, self.min_distance

    #     # 1) 前方近障仍尝试前进 => 大惩罚（距离越近惩罚越大）
    #     if action == 0 and front < 0.6:
    #         reward -= (0.6 - front) * 3.0   # 系数可在 2.0~5.0 间调

    #     # 2) 朝更空的一侧转弯 => 小奖励（只给正向差值）
    #     gap_bias = 0.0
    #     if action == 1:           # 左转
    #         gap_bias = (left - right)
    #     elif action == 2:         # 右转
    #         gap_bias = (right - left)
    #     reward += 0.2 * max(0.0, gap_bias)  # 0.1~0.3 可调

    #     # 3) 靠近障碍物的全局惩罚（保留）
    #     if self.min_distance < 0.5:
    #         reward -= (0.5 - self.min_distance) * 2.0

    #     # 4) 降低旋转惩罚，保留一点点惯性约束
    #     if action in [1, 2]:
    #         reward -= 0.005   # 原来是 0.02

    #     # --- 抗抖动项：左右来回切换给小惩罚 ---
    #     if prev_action in (1, 2) and action in (1, 2) and prev_action != action:
    #         reward -= 0.08   # 0.05~0.12 之间可调；先用 0.08


    #     # 5) 轻时间惩罚，鼓励高效
    #     reward -= 0.05

    #     # --- 终止条件 ---
    #     if current_distance < 0.3:
    #         reward += 30.0
    #         done = True
    #         self.last_reason = "reach"
    #         self.episode_success = True
    #         self.get_logger().info("reach the end")
    #     elif self.min_distance < 0.2:
    #         reward -= 30.0
    #         done = True
    #         self.last_reason = "crush"
    #         self.episode_crashes += 1
    #         self.get_logger().info("crush")
    #     elif self.step_count >= self.max_steps:
    #         reward -= 10.0
    #         done = True
    #         self.last_reason = "overtime"
    #         self.get_logger().info("overtime")
    #     else:
    #         done = False
    #         self.last_reason = "continue"

    #     self.prev_distance = current_distance

    #     return reward, done


    def compute_reward(self, action, prev_action=None):
        """
        强化避障 & 防卡死 的奖励塑形：
        - 进度为主，朝向为辅（弱化）
        - 三扇区引导：前方近障+前进重罚；转向更空一侧小奖；前进偏边扣分
        - 防抖：左右切换小罚
        - 防卡死：短窗口内进度几乎为零且前方不空 => 小罚
        """
        # ---------- 目标进度 & 朝向 ----------
        cur_dist = self.compute_distance_to_goal()
        if self.prev_distance is None:
            self.prev_distance = cur_dist
        progress = self.prev_distance - cur_dist      # >0 表示朝目标前进
        self.prev_distance = cur_dist

        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        goal_angle = math.atan2(dy, dx)
        heading_err = (goal_angle - self.orientation + math.pi) % (2*math.pi) - math.pi

        # 进度奖励（主）+ 弱化朝向项（辅）
        reward = 8.0 * progress + 0.25 * math.cos(heading_err)   # cos 更平滑，权重大幅低于进度

        # ---------- 激光三扇区 ----------
        scan = np.asarray(self.scan_state, dtype=float) if hasattr(self, "scan_state") else np.array([self.min_distance])
        n = len(scan)
        if n >= 5:
            mid = n // 2
            front = np.min(scan[max(0, mid-1):min(n, mid+2)])  # 取中间3束更稳
            left  = np.min(scan[mid:]) if mid < n else float('inf')
            right = np.min(scan[:mid]) if mid > 0 else float('inf')
        else:
            front = self.min_distance
            left = right = self.min_distance

        # 阈值可按地图再调
        FRONT_SOFT = 0.75      # 前方“拥挤”阈值（软惩罚）
        FRONT_HARD = 0.55      # 前进禁区阈值（硬惩罚更强）
        SIDE_SAFE  = 0.35      # 靠边阈值（前进时离一侧太近就扣分）

        # 1) 前方近障仍前进 => 重罚（硬/软两档）
        if action == 0 and front < FRONT_SOFT:
            k = 5.0 if front < FRONT_HARD else 3.0
            reward -= k * (FRONT_SOFT - front)  # 离得越近，扣得越多

        # 2) 向更空的一侧转弯 => 小奖励（只给正向差值）
        if action in (1, 2):
            gap_bias = (left - right) if action == 1 else (right - left)
            reward += 0.25 * max(0.0, gap_bias)   # 0.15~0.35 可调

        # 3) 前进时“偏边”扣分（鼓励居中通过）
        if action == 0:
            side_min = min(left, right)
            # 离墙过近时线性扣分；也可用 |left-right| 惩罚偏位，这里选“安全距离”优先
            if side_min < SIDE_SAFE:
                reward -= 1.5 * (SIDE_SAFE - side_min)

        # 4) 全局安全感（与最近障碍的距离）——温和惩罚
        if self.min_distance < 0.50:
            reward -= 2.0 * (0.50 - self.min_distance)

        # ---------- 防抖（左右来回切换罚） ----------
        if prev_action in (1, 2) and action in (1, 2) and prev_action != action:
            reward -= 0.10   # 0.08~0.12；抖得狠就加大

        # ---------- 防卡死（短窗口内进度≈0 且前方不空） ----------
        self.progress_win.append(progress)
        if len(self.progress_win) == self.progress_win.maxlen:
            win_prog = sum(self.progress_win)
            # 如果窗口内几乎没进步 + 前面不是“完全通畅”，给小罚推动策略切换
            if win_prog < 0.02 and front < 1.0:
                reward -= 0.20   # 可逐步递增（比如再多次触发则叠加），此处先固定

        # ---------- 轻时间惩罚 ----------
        reward -= 0.03

        # ---------- 终止判定 ----------
        if cur_dist < 0.30:
            reward += 30.0
            done = True
            self.last_reason = "reach"
            self.episode_success = True
            self.get_logger().info("reach the end")
        elif self.min_distance < 0.20:
            reward -= 30.0
            done = True
            self.last_reason = "crush"
            self.episode_crashes += 1
            self.get_logger().info("crush")
        elif self.step_count >= self.max_steps:
            reward -= 10.0
            done = True
            self.last_reason = "overtime"
            self.get_logger().info("overtime")
        else:
            done = False
            self.last_reason = "continue"

        return reward, done


    def is_violation(self):
        return self.min_distance < 0.2


# class NoMonitoringEnv(GazeboEnvironment):
#     def step(self, action):
#         """Discrete action: 0=forward, 1=left, 2=right"""
#         if action == 0:
#             self.base_step(0.4, 0.0)
#         elif action == 1:
#             self.base_step(0.0, 0.5)
#         elif action == 2:
#             self.base_step(0.0, -0.5)

#         self.step_count += 1
#         reward, done = self.compute_reward(action)
#         obs = self.get_observation()
#         return obs, reward, done, {}

class NoMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        if action == 0:  # 前进
            self.base_step(0.4, 0.0,  duration_sim=0.25, control_hz=40)
        elif action == 1:  # 左转
            self.base_step(0.0, 0.6,  duration_sim=0.20, control_hz=40)
        elif action == 2:  # 右转
            self.base_step(0.0, -0.6, duration_sim=0.20, control_hz=40)

        self.step_count += 1

        # ...
        prev_action = self.last_action          # 记一下之前的动作
        reward, done = self.compute_reward(action, prev_action)  # 传入上一步动作
        self.last_action = action               # 更新为当前动作
        # ...
        obs = self.get_observation()

        info = {
            "reason": self.last_reason,                           # reach/crush/overtime/continue
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
        if self.is_violation() and action == 0:  # Trying to move forward into obstacle
            self.get_logger().warn("Active Monitor: Stopping action due to violation!")
            action = 1  # Replace with turn left
        return super().step(action)
