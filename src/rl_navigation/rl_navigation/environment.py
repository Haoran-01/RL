# /rl_navigation/environment.py
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
from std_msgs.msg import UInt32  # <-- NEW: subscribe violation_count
import numpy as np
import math
import time
from collections import deque


class GazeboEnvironment(Node):
    def __init__(self, node_name='rl_env'):
        super().__init__(node_name)

        # ------- ROS IO -------
        # 学术对照：将原始动作发到 /cmd_vel_raw，由 monitor/cmd_guard 再转发到 /cmd_vel
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # <-- NEW: 订阅 ROSMonitoring/cmd_guard 报告的累计违规次数
        self.violation_count = 0
        self.prev_violation_count = 0
        self.create_subscription(UInt32, '/monitor_rl/violation_count', self.violation_cb, 10)

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
        self.progress_win = deque(maxlen=40)
        self.heading_win  = deque(maxlen=40)
        self.action_win   = deque(maxlen=40)

        # Services
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.reset_client = self.create_client(Empty, '/reset_simulation')


        # use_sim_time
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
    def violation_cb(self, msg: UInt32):
        self.violation_count = int(msg.data)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.orientation = self.quaternion_to_yaw(orientation_q)

    def scan_callback(self, msg: LaserScan):
        scan = np.array(msg.ranges, dtype=float)
        rng_max = getattr(msg, "range_max", 3.5) or 3.5
        scan[~np.isfinite(scan)] = rng_max
        scan[scan <= 0.0] = rng_max
        scan = np.clip(scan, 0.0, rng_max)
        self.min_distance = float(np.min(scan))
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

        for _ in range(5):
            self.vel_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.01)


        # Reset sim
        self.get_logger().info('Resetting the Gazebo world...')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_simulation service...')
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('World reset completed.')

        # Respawn robot
        state = EntityState()
        state.name = "turtlebot3_burger"
        state.pose.position.x = -3.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0
        req = SetEntityState.Request(); req.state = state

        if self.set_state_client.wait_for_service(timeout_sec=1.0):
            future2 = self.set_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future2)

        # Clear internal states
        self.position = None
        self.orientation = None
        self.min_distance = float('inf')
        self.prev_distance = None
        self.step_count = 0
        self.episode_crashes = 0
        self.episode_success = False
        self.last_reason = "continue"
        self.last_action = None
        self.prev_front = None
        self.prev_heading_err = None
        self.progress_win.clear()
        self.heading_win.clear()
        self.action_win.clear()

        # Sync sensors
        self.get_logger().info('Waiting for initial sensor data...')
        while rclpy.ok() and (self.position is None or self.orientation is None or self.min_distance == float('inf')):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info('Initial sensor data received.')

        # Snapshot violation counter at episode start
        self.prev_violation_count = self.violation_count

        d0 = self.compute_distance_to_goal()
        if d0 < 0.3:
            self.get_logger().warn(f"Spawned too close to goal! Distance={d0:.2f}")

        return self.get_observation()

    # ===================== Low-level stepping =====================
    def base_step(self, linear, angular, duration_sim=0.25, control_hz=40):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        start = self.get_clock().now()
        period = 1.0 / control_hz
        while (self.get_clock().now() - start).nanoseconds * 1e-9 < duration_sim:
            self.vel_pub.publish(cmd)           # 发布到 /cmd_vel_raw
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(min(0.0005, period * 0.2))
        self.vel_pub.publish(Twist())           # 停车
        rclpy.spin_once(self, timeout_sec=0.0)

    # ===================== Reward =====================
    def compute_reward(self, action, prev_action=None):
        # --- 进度/朝向/安全 ---（完全沿用你原始 shaping）
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

        scan = np.asarray(self.scan_state, dtype=float)
        n = len(scan)
        if n >= 5:
            mid = n // 2
            front = float(np.min(scan[max(0, mid-1):min(n, mid+2)]))
            left  = float(np.min(scan[mid:])) if mid < n else float('inf')
            right = float(np.min(scan[:mid])) if mid > 0 else float('inf')
        else:
            front = float(self.min_distance); left = right = float(self.min_distance)

        SAFE_HEADING = 0.80
        FRONT_SOFT   = 0.75
        FRONT_HARD   = 0.55
        SIDE_SAFE    = 0.35
        EARLY_GRACE  = 3

        base_progress_reward = 5.0 * progress
        if cur_dist < 1.0:
            base_progress_reward *= 1.5
        reward = base_progress_reward

        if progress >= 0 and front >= SAFE_HEADING and abs(heading_err) < abs(getattr(self, "prev_heading_err", heading_err)):
            reward += 0.25 * math.cos(heading_err)

        if action == 0:
            if self.prev_front is not None:
                df = front - self.prev_front
                reward += 0.4 * max(0.0, df)
                reward -= 0.6 * max(0.0, -df)
            side_min = min(left, right)
            if side_min < SIDE_SAFE:
                reward -= 3.0 * (SIDE_SAFE - side_min)

        if action == 0 and front < FRONT_SOFT:
            k = 5.0 if front < FRONT_HARD else 3.0
            reward -= k * (FRONT_SOFT - front)

        turn_bonus_factor = 1.0 if progress >= 0 else 0.0
        if action == 1:
            reward += turn_bonus_factor * 0.25 * max(0.0, (left - right))
        elif action == 2:
            reward += turn_bonus_factor * 0.25 * max(0.0, (right - left))

        if self.last_action in (1,2) and action in (1,2) and self.last_action != action:
            reward -= 0.3

        if front >= SAFE_HEADING:
            reward -= 0.15 * d_heading

        if self.min_distance < 0.50:
            dist_penalty = 2.0 * (0.50 - self.min_distance) ** 1.5
            reward -= dist_penalty

        self.progress_win.append(progress)
        if len(self.progress_win) == self.progress_win.maxlen and sum(self.progress_win) < 0.03:
            reward -= 0.2

        # --- 终止 ---
        if cur_dist < 0.30:
            reward += 40.0; done = True; self.last_reason="reach"; self.episode_success=True; self.get_logger().info("reach the end")
        elif (self.step_count > EARLY_GRACE) and (self.min_distance < 0.20):
            reward -= 30.0; done = True; self.last_reason="crush"; self.episode_crashes += 1; self.get_logger().info("crush")
        else:
            # stuck 检测（同你原版逻辑）
            self.progress_win.append(progress)
            self.heading_win.append(heading_err)
            self.action_win.append(action)

            GRACE_STEPS_FOR_STUCK = 60
            PROG_THRESH = 0.05
            HEADING_JITTER = 0.12
            TURN_RATIO_MAX = 0.30
            FRONT_IMPROVE = 0.05

            if (self.step_count > GRACE_STEPS_FOR_STUCK 
                and len(self.progress_win) == self.progress_win.maxlen):

                total_prog = float(sum(self.progress_win))
                if len(self.heading_win) >= 2:
                    max_d_heading = max(abs(b-a) for a, b in zip(self.heading_win, list(self.heading_win)[1:]))
                else:
                    max_d_heading = 0.0

                turns = sum(1 for a in self.action_win if a in (1, 2))
                turn_ratio = turns / float(len(self.action_win))

                front_now = front
                front_prev = self.prev_front if self.prev_front is not None else front_now
                front_improve = max(0.0, front_now - front_prev)

                stuck = (
                    total_prog < PROG_THRESH and
                    max_d_heading < HEADING_JITTER and
                    turn_ratio < TURN_RATIO_MAX and
                    front_improve < FRONT_IMPROVE
                )

                if stuck:
                    reward -= 5.0
                    done = True
                    self.last_reason = "stuck"
                    self.get_logger().info(
                        f"early stop: stuck | prog={total_prog:.3f}/{self.progress_win.maxlen} "
                        f"| d_head_max={max_d_heading:.3f} | turn_ratio={turn_ratio:.2f} | d_front={front_improve:.3f}"
                    )
                    self.prev_front = front
                    return reward, done

            if self.step_count >= self.max_steps:
                reward -= 10.0; done = True; self.last_reason = "overtime"; self.get_logger().info("overtime")
            else:
                done = False; self.last_reason = "continue"

        self.prev_front = front
        return reward, done

    def is_violation(self):
        return self.min_distance < 0.2

# ===================== Envs =====================
class NoMonitoringEnv(GazeboEnvironment):
    def step(self, action):
        """0=forward, 1=left, 2=right"""
        if action == 0:
            self.base_step(0.25, 0.0, duration_sim=0.20, control_hz=40)
        elif action == 1:
            self.base_step(0.15, 0.55, duration_sim=0.18, control_hz=40)
        elif action == 2:
            self.base_step(0.15, -0.55, duration_sim=0.18, control_hz=40)

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
            "monitor_delta_v": 0,              # baseline: 无监控增量
        }
        self.last_action = action
        return obs, reward, done, info


class PassiveMonitoringEnv(GazeboEnvironment):
    """只提示，不改变动作，也不注入惩罚（可用于“被动观察”组）"""
    def step(self, action):
        if self.is_violation():
            self.get_logger().warn("Passive Monitor: Violation detected!")
        return super().step(action)


class ActiveMonitoringEnv(GazeboEnvironment):
    """
    推荐用于“Active+Penalty”实验：动作仍由 ROSMonitoring/cmd_guard 拦截，
    这里额外在 reward 中加入“监控增量违规”的惩罚，以对齐外部拦截的学习信号。
    可选：在前方过近时对“直行”做轻微替换（与论文常见设置一致），但真正的硬拦截仍交给监控。
    """
    VIOLATION_PENALTY = 2.0   # 每条新违规的惩罚强度，可调参
    USE_SOFT_OVERRIDE = False   # 可切换是否在环境侧做软替换

    def step(self, action):
        # （可选）软替换：仅在前方过近时把直行替换成随机转向；真正的安全兜底靠监控
        if self.USE_SOFT_OVERRIDE:
            front = float(self.scan_state[len(self.scan_state) // 2])
            if action == 0 and front < 0.35:
                self.get_logger().warn("ActiveMonitor(Env): block forward -> soft turn")
                action = 1 if np.random.rand() < 0.5 else 2

        # 记录监控计数（步前）
        v_before = self.violation_count

        # 执行动作（动作发往 /cmd_vel_raw；若监控判为不安全，将在中间被 cmd_guard 刹停）
        if action == 0:
            self.base_step(0.25, 0.0, duration_sim=0.20, control_hz=40)
        elif action == 1:
            self.base_step(0.0, 0.9, duration_sim=0.18, control_hz=40)
        elif action == 2:
            self.base_step(0.0, -0.9, duration_sim=0.18, control_hz=40)

        self.step_count += 1

        # 计算环境基础奖励
        reward, done = self.compute_reward(action)

        # 追加“监控增量违规”惩罚

        # 等待一个小窗口收齐 monitor 消息（用仿真时钟，不受 wall time 影响）
        wait_until = self.get_clock().now().nanoseconds * 1e-9 + 0.05  # 50ms 仿真时间
        v_after = self.violation_count
        while v_after == v_before and (self.get_clock().now().nanoseconds * 1e-9) < wait_until:
            rclpy.spin_once(self, timeout_sec=0.0)

            # 再读一次
            v_after = self.violation_count

        # for _ in range(3):  # 多 spin 几次，给 verdict 机会传到
        #     rclpy.spin_once(self, timeout_sec=0.02)
        # v_after = self.violation_count

        delta_v = max(0, int(v_after - v_before))
        delta = 0

        # 新回合头两步的溢出豁免（且环境并不危险）
        front = float(self.scan_state[len(self.scan_state)//2])
        if self.step_count <= 5 and delta_v > 0 and front > 0.6:
            self.get_logger().info("Ignore spillover verdict at episode start")
            delta_v = 0


        if delta_v > 0:
            penalty = self.VIOLATION_PENALTY * float(delta_v)
            delta = 1
            reward -= penalty
            self.get_logger().warn(f"Monitor penalty: -{penalty:.1f} for {delta_v} new violation(s)")
            

        obs = self.get_observation()
        info = {
            "reason": self.last_reason,
            "distance_to_goal": float(self.compute_distance_to_goal()),
            "min_laser": float(self.min_distance),
            "step_count": int(self.step_count),
            "violation": bool(self.is_violation()),
            "success": bool(self.episode_success),
            "crashes_in_episode": int(self.episode_crashes),
            "monitor_delta_v": int(delta),   # <-- 关键监控指标，便于画图/统计
        }
        self.last_action = action
        return obs, reward, done, info
