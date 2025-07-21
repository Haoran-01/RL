#!/usr/bin/env python3
import rclpy
import numpy as np
from environments import NoMonitoringEnv, PassiveMonitoringEnv, ActiveMonitoringEnv
import time

class QLearningAgent:
    def __init__(self, actions, alpha=0.1, gamma=0.9, epsilon=0.2):
        self.q_table = {}
        self.actions = actions
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon

    def get_q(self, state, action):
        return self.q_table.get((state, action), 0.0)

    def choose_action(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.choice(self.actions)
        q_values = [self.get_q(state, a) for a in self.actions]
        max_q = max(q_values)
        return self.actions[q_values.index(max_q)]

    def learn(self, state, action, reward, next_state):
        prev_q = self.get_q(state, action)
        max_future_q = max([self.get_q(next_state, a) for a in self.actions])
        new_q = prev_q + self.alpha * (reward + self.gamma * max_future_q - prev_q)
        self.q_table[(state, action)] = new_q


def compute_reward(env, state, goal_x, goal_y, tolerance=0.3):
    x, y, _ = state
    if abs(x - goal_x) < tolerance and abs(y - goal_y) < tolerance:
        return +100, True
    if env.is_violation():
        return -100, True
    return -1, False


def main():
    rclpy.init()
    # 选择实验组
    env = NoMonitoringEnv()
    # env = PassiveMonitoringEnv()
    # env = ActiveMonitoringEnv()

    agent = QLearningAgent(actions=['forward', 'left', 'right'])
    goal_x, goal_y = -2.97014, -3.32553
    episodes = 50

    for ep in range(episodes):
        env.reset()
        time.sleep(1.0)
        state = env.get_state()
        done = False
        total_reward = 0

        while not done and rclpy.ok():
            action = agent.choose_action(state)
            if action == 'forward':
                env.step(linear=0.2, angular=0.0)
            elif action == 'left':
                env.step(linear=0.0, angular=0.5)
            elif action == 'right':
                env.step(linear=0.0, angular=-0.5)

            next_state = env.get_state()
            reward, done = compute_reward(env, next_state, goal_x, goal_y)
            # update the q value
            agent.learn(state, action, reward, next_state)
            state = next_state
            total_reward += reward

        print(f"Episode {ep+1}: Total Reward = {total_reward}")

    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
