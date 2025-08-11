import rclpy
from rl_navigation.environment import NoMonitoringEnv  # 或切换 ActiveMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent
import torch
import numpy as np
import matplotlib.pyplot as plt
import os
import csv
from datetime import datetime


def train_ddqn(episodes=2000, max_steps=500):
    rclpy.init()
    env = NoMonitoringEnv()

    agent = DDQNAgent(state_dim=13, action_dim=3)  # 3(pos+yaw)+10 beams

   # 如果有旧模型，加载
    if os.path.exists("ddqn_model_final_v3.pth"):
        agent.load("ddqn_model_final_v3.pth")
        agent.epsilon = 1.0 # 可选，手动调整起始探索率

    # 线性衰减设置：600 个 episode 从 1.0 -> 0.05
    EPS_DECAY_EPISODES = 1600
    EPS_START = 0.10
    EPS_END = agent.epsilon_min

    # 目标网同步间隔（按训练步计数，不是环境步）
    TARGET_UPDATE_EVERY = 2000

    # 日志
    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs("logs", exist_ok=True)
    step_log_path = f"logs/steps_{run_tag}.csv"
    epi_log_path = f"logs/episodes_{run_tag}.csv"

    with open(step_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["episode", "step", "action", "reward", "epsilon",
                    "distance_to_goal", "min_laser", "done", "reason", "total_reward_so_far"])
    with open(epi_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["episode", "total_reward", "steps", "success", "crashes", "avg_reward_per_step", "epsilon_end", "reason"])

    global_train_steps = 0  # 计数“训练更新”的步数
    
    for episode in range(episodes):
        obs = env.reset()
        total_reward = 0.0
        last_info = {"reason": "overtime"}
        crashes_this_episode = 0

        # 线性衰减 epsilon
        frac = min(1.0, (episode + 1) / EPS_DECAY_EPISODES)
        agent.epsilon = EPS_START + (EPS_END - EPS_START) * frac

        for step in range(max_steps):
            if obs is None:
                env.get_logger().warn("Observation is None. Skipping step...")
                last_info = {"reason": "continue"}
                continue

            action = agent.act(obs)
            # ★ 只调用一次 env.step
            next_obs, reward, done, info = env.step(action)

            agent.remember(obs, action, reward, next_obs, done)
            prev_update_counter = agent.update_counter
            agent.replay()

            # 若这一步发生了训练，累加训练步
            if agent.update_counter != prev_update_counter:
                global_train_steps += 1
                if global_train_steps % TARGET_UPDATE_EVERY == 0:
                    agent.update_target()


            obs = next_obs
            total_reward += reward
            last_info = info if info is not None else {"reason": "continue"}

            # 逐步日志
            with open(step_log_path, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    episode + 1, step + 1, int(action), float(reward), float(agent.epsilon),
                    float(info.get("distance_to_goal", np.nan)),
                    float(info.get("min_laser", np.nan)),
                    int(done), info.get("reason", ""), float(total_reward)
                ])

            if info.get("reason") == "crush":
                crashes_this_episode += 1

            if done:
                break

        agent.update_target()

        final_reason = last_info.get("reason", "overtime")
        success_flag = 1 if final_reason == "reach" else 0
        steps_taken = step + 1 if 'step' in locals() else 0
        avg_r = total_reward / max(1, steps_taken)

        print(f"Episode {episode+1}/{episodes} | Total Reward: {total_reward:.2f} | Steps: {steps_taken} | Reason: {final_reason}")

        with open(epi_log_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                episode + 1, float(total_reward), steps_taken, success_flag,
                int(crashes_this_episode), float(avg_r), float(agent.epsilon), final_reason
            ])

    torch.save(agent.q_net.state_dict(), "ddqn_model_final_v3.pth")

    env.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    train_ddqn()
