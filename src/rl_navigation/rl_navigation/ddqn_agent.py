import torch
import torch.nn as nn
import torch.optim as optim
import random
import numpy as np
from collections import deque

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class DDQN(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DDQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )

    def forward(self, x):
        return self.net(x)

class DDQNAgent:
    def __init__(self, state_dim, action_dim):
        self.q_net = DDQN(state_dim, action_dim).to(device)
        self.target_net = DDQN(state_dim, action_dim).to(device)
        self.target_net.load_state_dict(self.q_net.state_dict())
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=0.001)
        self.memory = deque(maxlen=10000)
        self.gamma = 0.99
        self.batch_size = 64
        self.epsilon = 1.0
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.99
        self.loss_history = []

    def act(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, 2)  # 3 actions
        state = torch.FloatTensor(state).unsqueeze(0).to(device)
        with torch.no_grad():
            q_values = self.q_net(state)
        return torch.argmax(q_values).item()

    def remember(self, s, a, r, s_, done):
        self.memory.append((s, a, r, s_, done))


    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        batch = random.sample(self.memory, self.batch_size)
        s, a, r, s_, done = zip(*batch)

        s = torch.from_numpy(np.array(s, dtype=np.float32)).to(device)
        a = torch.LongTensor(a).unsqueeze(1).to(device)
        r = torch.FloatTensor(r).unsqueeze(1).to(device)
        s_ = torch.from_numpy(np.array(s_, dtype=np.float32)).to(device)
        done = torch.FloatTensor(done).unsqueeze(1).to(device)

        q_values = self.q_net(s).gather(1, a)
        next_actions = torch.argmax(self.q_net(s_), dim=1, keepdim=True)
        next_q_values = self.target_net(s_).gather(1, next_actions)
        target = r + (1 - done) * self.gamma * next_q_values

        loss = nn.MSELoss()(q_values, target.detach())
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        self.loss_history.append(loss.item())


    def update_target(self):
        self.target_net.load_state_dict(self.q_net.state_dict())

    def save(self, filename):
        torch.save(self.q_net.state_dict(), filename)

    def load(self, filename):
        self.q_net.load_state_dict(torch.load(filename, map_location=device))
