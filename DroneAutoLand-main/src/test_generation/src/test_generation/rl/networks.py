import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from einops.layers.torch import Rearrange, Reduce
import numpy as np
from einops import rearrange, reduce, repeat
import random
# import gym
import time
# from environment import CustomEnv
# DQN网络

import torch
import torch.nn as nn

class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 24),
            nn.ReLU(),
            nn.Linear(24, output_dim)
        )

    def forward(self, x):
        return self.fc(x)


class DQNAgent:
    def __init__(self, input_dim, action_space):
        self.q_network = DQN(input_dim, len(action_space)).cuda()
        self.target_network = DQN(input_dim, len(action_space)).cuda()
        self.target_network.load_state_dict(self.q_network.state_dict())
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=0.001)
        self.criterion = nn.MSELoss()

        self.action_space = action_space
        self.epsilon = 0.9
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.gamma = 0.99

    def get_action(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.choice(self.action_space)
        q_values = self.q_network(torch.FloatTensor(state).cuda()).cuda()
        return self.action_space[torch.argmax(q_values).item()]

    def evaluate(self, state):
        self.q_network.load_state_dict(torch.load('agent_q_network_weights.pth'))
        q_values = self.q_network(torch.FloatTensor(state).cuda()).cuda()
        return self.action_space[torch.argmax(q_values).item()]

    def train(self, experiences):
        states, actions, rewards, next_states, done = zip(*experiences)
        # states = torch.FloatTensor(states)
        states = torch.FloatTensor(states).cuda()
        actions = torch.LongTensor(actions).cuda()
        rewards = torch.FloatTensor(rewards).cuda()
        next_states = torch.FloatTensor(next_states).cuda()
        done = torch.LongTensor(done).cuda()
        # dones = torch.FloatTensor(dones)

        # Compute the Q values
        # print(states.shape)
        q_values = self.q_network(states).gather(1, actions.unsqueeze(-1))
        next_q_values = self.q_network(next_states).max(1)[0]
        target = rewards + (1-done)*self.gamma * next_q_values

        loss = self.criterion(q_values, target.unsqueeze(-1))
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        torch.save(self.q_network.state_dict(), '/home/linfeng/Documents/ICSE_2025_weights.pth')
        return loss.item()
    def update_target(self):
        self.target_network.load_state_dict(self.q_network.state_dict())

# class MultiHeadAttention(nn.Module):
#     def __init__(self, emb_size, num_heads: int = 8, dropout: float = 0.1):
#         super().__init__()
#         self.emb_size = emb_size
#         self.num_heads = num_heads
#         # 将查询、键和值融合到一个矩阵中
#         self.qkv = nn.Linear(emb_size, emb_size * 3)
#         self.att_drop = nn.Dropout(dropout)
#         self.projection = nn.Linear(emb_size, emb_size)

#     def forward(self, x):
#         # 分割num_heads中的键、查询和值
#         qkv = rearrange(self.qkv(x), "b n (h d qkv) -> (qkv) b h n d", h=self.num_heads, qkv=3)


#         queries, keys, values = qkv[0], qkv[1], qkv[2]
#         # print('queries', values.shape)
#         # 最后一个轴上求和
#         energy = torch.einsum('bhqd, bhkd -> bhqk', queries, keys)  # batch, num_heads, query_len, key_len

#         scaling = self.emb_size ** (1 / 2)
#         att = F.softmax(energy, dim=-1) / scaling

#         # print(att.shape)
#         att = self.att_drop(att)
#         # 在第三个轴上求和
#         out = torch.einsum('bhal, bhlv -> bhav ', att, values)
#         # print(out.shape)
#         out = rearrange(out, "b h n d -> b n (h d)")
#         out = self.projection(out)

#         return out

# class ResidualAdd(nn.Module):
#     def __init__(self, fn):
#         super().__init__()
#         self.fn = fn

#     def forward(self, x):
#         res = x
#         x = self.fn(x)
#         x += res
#         return x

# class FeedForwardBlock(nn.Sequential):
#     def __init__(self, emb_size: int, expansion: int = 4, drop_p: float = 0.):
#         super().__init__(
#             nn.Linear(emb_size, expansion * emb_size),
#             nn.GELU(),
#             nn.Dropout(drop_p),
#             nn.Linear(expansion * emb_size, emb_size),
#         )

# class TransformerEncoderBlock(nn.Sequential):
#     def __init__(self,
#                  emb_size,
#                  drop_p: float = 0.1,
#                  forward_expansion: int = 4,
#                  forward_drop_p: float = 0.1,
#                  ** kwargs):
#         super().__init__(
#             ResidualAdd(nn.Sequential(
#                 nn.LayerNorm(emb_size),
#                 MultiHeadAttention(emb_size, **kwargs),
#                 nn.Dropout(drop_p)
#             )),
#             ResidualAdd(nn.Sequential(
#                 nn.LayerNorm(emb_size),
#                 FeedForwardBlock(
#                     emb_size, expansion=forward_expansion, drop_p=forward_drop_p),
#                 nn.Dropout(drop_p)
#             )
#             ))

# class TransformerEncoder(nn.Sequential):
#     def __init__(self, depth, **kwargs):
#         super().__init__(*[TransformerEncoderBlock(**kwargs) for _ in range(6)],
#                          # nn.AvgPool2d(32, 256),


#                          )

# # # 示例
# # model = MultiHeadSelfAttention(embed_size=32, heads=8)
# # x = torch.rand((32, 100, 32))  # 32 is batch size, 100 is sequence length, 256 is embedding size
# # out = model(x, x, x, None)  # Q=K=V=x
# # print(out.shape)  # torch.Size([32, 100, 256])



# class DuelingNetwork(nn.Module):
#     def __init__(self, input_dim, num_actions):
#         super(DuelingNetwork, self).__init__()

#         # Fully connected layer followed by ReLU
#         self.fcagent1 = nn.Linear(input_dim*3, 32)
#         self.fcagent2 = nn.Linear(input_dim*3, 32)
#         self.fcagent3 = nn.Linear(input_dim*3, 32)
#         self.relu1 = nn.ReLU()

#         self.attention = TransformerEncoder(6, emb_size=32)
#         # Dueling Network
#         self.value_stream = nn.Linear(32, 1)
#         self.advantage_stream = nn.Linear(32, num_actions)

#         self.mean = Reduce('B T H -> B H ', reduction='mean')

#     def forward(self, x1, x2, x3):
#         # Pass through first fully connected layer and ReLU

#         x1_attention = torch.cat((x1, x2, x3), dim=1)
#         x2_attention = torch.cat((x2, x1, x3), dim=1)
#         x3_attention = torch.cat((x3, x1, x2), dim=1)

#         # print(x1_attention.shape)

#         x1 = self.fcagent1(x1_attention)
#         x2 = self.fcagent2(x2_attention)
#         x3 = self.fcagent3(x3_attention)
      

#         x1 = self.relu1(x1)
#         x2 = self.relu1(x2)
#         x3 = self.relu1(x3) 
      

        
#         # print(x3_attention.reshape(-1).shape)

#         # x1_attention = self.mean(self.attention(x1_attention))
#         # x2_attention = self.mean(self.attention(x2_attention))
#         # x3_attention = self.mean(self.attention(x3_attention))
#         # print(x3_attention.shape)


#         # Dueling Part
#         value1 = self.value_stream(x1)
#         advantages1 = self.advantage_stream(x1)

#         value2 = self.value_stream(x2)
#         advantages2 = self.advantage_stream(x2)

#         value3 = self.value_stream(x3)
#         advantages3 = self.advantage_stream(x3)

#         # Combine value and advantages to get Q-values
#         qvals1 = value1 + (advantages1 - advantages1.mean())
#         qvals2 = value2 + (advantages2 - advantages2.mean())
#         qvals3 = value3 + (advantages3 - advantages3.mean())
        
#         return qvals1, qvals2, qvals3


# a = DuelingNetwork(2,4)
# x = torch.rand((32, 2))
# y = torch.rand((32, 2))
# z = torch.rand((32, 2))
# # b = a(x,y,z)
# # print(b.shape)
