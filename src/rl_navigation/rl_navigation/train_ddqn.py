import rclpy
from rl_navigation.environment import NoMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent
import torch
import numpy as np
import matplotlib.pyplot as plt
import os
import csv
from datetime import datetime


def train_ddqn(episodes=2000, max_steps=1000):
    rclpy.init()
    env = NoMonitoringEnv()
    agent = DDQNAgent(state_dim=13, action_dim=3)  # 3 pos+orientation + 10 laser beams

    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    step_log_path = f"logs/steps_{run_tag}.csv"
    epi_log_path  = f"logs/episodes_{run_tag}.csv"
    os.makedirs("logs", exist_ok=True)

    # 写表头
    with open(step_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["episode","step","action","reward","epsilon",
                    "distance_to_goal","min_laser","done","reason","total_reward_so_far"])

    with open(epi_log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["episode","total_reward","steps","success","crashes","avg_reward_per_step","epsilon_end"])


    # 如果有旧模型，加载
    if os.path.exists("ddqn_model_final_v1.pth"):
        agent.load("ddqn_model_final_v1.pth")
        agent.epsilon = 1.0  # 可选，手动调整起始探索率


    for episode in range(episodes):
        obs = env.reset()
        total_reward = 0
        step = 0

        crashes_this_episode = 0
        last_info = {"reason": "overtime"}  # 合理的默认：如果没触发done，就按超时处理
        last_done = False

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay


        for step in range(max_steps):
            if obs is None:
                env.get_logger().warn("Observation is None. Skipping step...")
                last_info = {"reason": "continue"}
                continue

            action = agent.act(obs)
            next_obs, reward, done, info = env.step(action) 
            agent.remember(obs, action, reward, next_obs, done)
            agent.replay()
            obs = next_obs
            total_reward += reward

            last_info = info if info is not None else {"reason": "continue"}
            last_done = bool(done)

            with open(step_log_path, "a", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    episode+1, step+1, action, float(reward), float(agent.epsilon),
                    float(info.get("distance_to_goal", float("nan"))),
                    float(info.get("min_laser", float("nan"))),
                    int(done), info.get("reason",""),
                    float(total_reward)
                ])

            if info.get("reason") == "crush":
                crashes_this_episode += 1

            if done:
                break

        agent.update_target()
        print(f"Episode {episode+1}/{episodes} | Total Reward: {total_reward:.2f} | Steps: {step+1}")

        # ★ 每回合汇总
        final_reason = last_info.get("reason", "overtime")
        success_flag = 1 if final_reason == "reach" else 0

        success_flag = 1 if info.get("reason") == "reach" else 0
        avg_r = total_reward / (step+1 if step>=0 else 1)
        with open(epi_log_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                episode+1, float(total_reward), step+1, success_flag,
                crashes_this_episode, float(avg_r), float(agent.epsilon)
            ])


    

    agent.save("ddqn_model_final_v1.pth")

    env.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    train_ddqn()
