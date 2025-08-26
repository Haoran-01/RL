import pandas as pd
import matplotlib.pyplot as plt

# ===== 1) 读取训练日志（按你给的结构）=====
path = "episodes_20250826_141837_none.csv"
df = pd.read_csv(path)

# （可选）清理异常 reward：低于 -200 的行（只在内存中过滤，不改原文件）
df = df[df["total_reward"] >= -150].copy()

# ===== 2) Reward 平滑与趋势 =====
df['total_reward_smooth'] = df['total_reward'].rolling(window=5, min_periods=1).mean()

plt.figure(figsize=(10, 5))
plt.plot(df['episode'], df['total_reward'], label='Total Reward', alpha=0.6)
plt.plot(df['episode'], df['total_reward_smooth'], label='Smoothed (window=5)', linestyle='--', linewidth=2)
plt.xlabel('Episode')
plt.ylabel('Total Reward')
plt.title('Reward per Episode (cleaned: total_reward >= -200)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# ===== 3) 终止原因占比：overtime / crush / stuck / reach =====
reason_norm = (
    df['reason']
    .astype(str)
    .str.strip()
    .str.lower()
)

# 仅统计四类指定原因；其他归为 other（如果没有就显示 0）
cats = ['reach', 'crush', 'stuck', 'overtime']
reason_counts = reason_norm.map(lambda x: x if x in cats else 'other').value_counts()
reason_counts = reason_counts.reindex(cats + ['other'], fill_value=0)
reason_pct = (reason_counts / reason_counts.sum() * 100).round(2)

print("Reason percentage (%):")
print(reason_pct.to_string())

plt.figure(figsize=(8, 4))
(reason_pct.drop(labels=['other']) if reason_pct['other'] == 0 else reason_pct).plot(kind='bar')
plt.ylabel('Percentage (%)')
plt.title('Episode Termination Reasons')
plt.grid(axis='y', linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()

# ===== 4) 每个批次（episode）的步数与 Δsteps 趋势 =====
# 你的 CSV 已经提供了 steps（每集步数），直接使用
steps_df = df[['episode', 'steps']].dropna().sort_values('episode').set_index('episode')
steps_df['steps'] = steps_df['steps'].astype(int)
steps_df['steps_delta'] = steps_df['steps'].diff()

print("\nPer-episode steps (head):")
print(steps_df.head().to_string())

# 图：steps 趋势
plt.figure(figsize=(10, 4))
plt.plot(steps_df.index, steps_df['steps'])
plt.xlabel('Episode')
plt.ylabel('Steps')
plt.title('Steps per Episode')
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()

# 图：Δsteps 趋势
plt.figure(figsize=(10, 4))
plt.plot(steps_df.index, steps_df['steps_delta'])
plt.xlabel('Episode')
plt.ylabel('Δ Steps vs Prev Episode')
plt.title('Change in Steps per Episode')
plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()
plt.show()
