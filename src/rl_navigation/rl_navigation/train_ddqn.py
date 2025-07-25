import rclpy
from rl_navigation.environment import NoMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent
import torch
import numpy as np
import matplotlib.pyplot as plt
import os

def train_ddqn(episodes=500, max_steps=1000):
    rclpy.init()
    env = NoMonitoringEnv()
    agent = DDQNAgent(state_dim=13, action_dim=3)  # 3 pos+orientation + 10 laser beams

    # 如果有旧模型，加载
    if os.path.exists("ddqn_model_v4.pth"):
        agent.load("ddqn_model_v4.pth")
        agent.epsilon = 1.0  # 可选，手动调整起始探索率


    for episode in range(episodes):
        obs = env.reset()
        total_reward = 0

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay


        for step in range(max_steps):
            if obs is None:
                env.get_logger().warn("Observation is None. Skipping step...")
                continue

            action = agent.act(obs)
            next_obs, reward, done, _ = env.step(action)
            agent.remember(obs, action, reward, next_obs, done)
            agent.replay()
            obs = next_obs
            total_reward += reward

            if done:
                print(step)
                break

        agent.update_target()
        print(f"Episode {episode+1}/{episodes} | Total Reward: {total_reward:.2f} | Steps: {step+1}")

    with open("loss_log_v5.txt", "w") as f:
        for l in agent.loss_history:
            f.write(f"{l}\n")

    
    plt.plot(agent.loss_history)
    plt.xlabel("Training Steps")
    plt.ylabel("Loss")
    plt.title("Loss Curve")
    plt.savefig("loss_curve_v5.png")

    agent.save("ddqn_model_v5.pth")

    env.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    train_ddqn()
