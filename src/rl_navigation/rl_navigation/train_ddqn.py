# /rl_navigation/train_ddqn.py
import argparse
import rclpy
from rl_navigation.environment import NoMonitoringEnv, PassiveMonitoringEnv, ActiveMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent
import torch
import numpy as np
import os, csv
from datetime import datetime


def make_env(mode: str):
    if mode == "none":
        return NoMonitoringEnv()
    elif mode == "passive":
        return PassiveMonitoringEnv()
    elif mode == "active":
        return ActiveMonitoringEnv()
    else:
        raise ValueError(f"Unknown mode: {mode} (choose from none|passive|active)")


def train_ddqn(episodes=1000, max_steps=1000, mode="none"):
    rclpy.init()
    env = make_env(mode)

    agent = DDQNAgent(state_dim=13, action_dim=3)  # 3(pos+yaw)+10 beams

    # 可选：加载旧模型
    if os.path.exists("ddqn_model_final_v2.pth"):
        agent.load("ddqn_model_final_v2.pth")
        agent.epsilon = 1.0

    # epsilon 线性衰减
    EPS_DECAY_EPISODES = 500
    EPS_START = 1.0
    EPS_END = agent.epsilon_min

    # 目标网同步间隔（按训练更新步计）
    TARGET_UPDATE_EVERY = 2000

    # 日志
    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{mode}"
    os.makedirs("logs", exist_ok=True)
    step_log_path = f"logs/steps_{run_tag}.csv"
    epi_log_path = f"logs/episodes_{run_tag}.csv"

    with open(step_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "episode", "step", "action", "reward", "epsilon",
            "distance_to_goal", "min_laser", "done", "reason",
            "total_reward_so_far", "monitor_delta_v"  # <-- 新增
        ])
    with open(epi_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "episode", "total_reward", "steps", "success", "crashes",
            "avg_reward_per_step", "epsilon_end", "reason"
        ])

    global_train_steps = 0

    for episode in range(episodes):
        obs = env.reset()
        total_reward = 0.0
        last_info = {"reason": "overtime"}
        crashes_this_episode = 0

        # epsilon 线性衰减
        frac = min(1.0, (episode + 1) / EPS_DECAY_EPISODES)
        agent.epsilon = EPS_START + (EPS_END - EPS_START) * frac

        for step in range(max_steps):
            if obs is None:
                env.get_logger().warn("Observation is None. Skipping step...")
                last_info = {"reason": "continue"}
                continue

            action = agent.act(obs)
            next_obs, reward, done, info = env.step(action)

            agent.remember(obs, action, reward, next_obs, done)
            prev_update_counter = agent.update_counter
            agent.replay()

            if agent.update_counter != prev_update_counter:
                global_train_steps += 1
                if global_train_steps % TARGET_UPDATE_EVERY == 0:
                    agent.update_target()

            obs = next_obs
            total_reward += reward
            last_info = info if info is not None else {"reason": "continue"}

            with open(step_log_path, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    episode + 1, step + 1, int(action), float(reward), float(agent.epsilon),
                    float(info.get("distance_to_goal", np.nan)),
                    float(info.get("min_laser", np.nan)),
                    int(done), info.get("reason", ""), float(total_reward),
                    int(info.get("monitor_delta_v", 0))  # <-- 新增
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

        print(f"[{mode}] Episode {episode+1}/{episodes} | "
              f"R: {total_reward:.2f} | Steps: {steps_taken} | Reason: {final_reason}")

        with open(epi_log_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                episode + 1, float(total_reward), steps_taken, success_flag,
                int(crashes_this_episode), float(avg_r), float(agent.epsilon), final_reason
            ])

    torch.save(agent.q_net.state_dict(), f"ddqn_model_{mode}.pth")

    env.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--max_steps", type=int, default=1000)
    parser.add_argument("--mode", type=str, default="none", choices=["none", "passive", "active"])
    args = parser.parse_args()
    train_ddqn(episodes=args.episodes, max_steps=args.max_steps, mode=args.mode)
