import pandas as pd
import matplotlib.pyplot as plt

# 读取训练日志
df = pd.read_csv("logs/episodes_20250814_124621.csv")

# 如果 CSV 中有列名包含空格或意外字符，可以先检查一下
# print(df.columns)

# 添加平滑处理（滑动平均）
df['total_reward_smooth'] = df['total_reward'].rolling(window=5, min_periods=1).mean()

# 绘图
plt.figure(figsize=(10, 5))
plt.plot(df['episode'], df['total_reward'], label='Total Reward', alpha=0.6)
plt.plot(df['episode'], df['total_reward_smooth'], label='Smoothed (window=5)', linestyle='--', linewidth=2)
plt.xlabel('Episode')
plt.ylabel('Total Reward')
plt.title('Reward per Episode')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
