import rclpy
from rl_navigation.environment import NoMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent
import torch
import numpy as np

def train_ddqn(episodes=100, max_steps=3500):
    rclpy.init()
    env = NoMonitoringEnv()
    agent = DDQNAgent(state_dim=13, action_dim=3)  # 3 pos+orientation + 10 laser beams

    for episode in range(episodes):
        obs = env.reset()
        total_reward = 0

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

    torch.save(agent.q_network.state_dict(), "ddqn_model.pth")
    env.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    train_ddqn(max_steps=3500)
