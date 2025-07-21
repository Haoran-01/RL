#!/usr/bin/env python3
import rclpy
from rl_navigation.environment import NoMonitoringEnv, PassiveMonitoringEnv, ActiveMonitoringEnv
from rl_navigation.ddqn_agent import DDQNAgent

def test(env_class, model_name, episodes=10):
    rclpy.init()
    env = env_class()
    obs = env.reset()
    state_dim = len(obs)
    action_dim = 3
    agent = DDQNAgent(state_dim, action_dim)
    agent.load(f"models/{model_name}")

    for ep in range(episodes):
        obs = env.reset()
        total_reward = 0
        done = False

        while not done:
            action = agent.act(obs)
            obs, reward, done, _ = env.step(action)
            total_reward += reward

        print(f"Test Episode {ep+1}: Total Reward = {total_reward:.2f}")

    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test(NoMonitoringEnv, "ddqn_no_monitor.pth")
    # test(PassiveMonitoringEnv, "ddqn_passive_monitor.pth")
    # test(ActiveMonitoringEnv, "ddqn_active_monitor.pth")
