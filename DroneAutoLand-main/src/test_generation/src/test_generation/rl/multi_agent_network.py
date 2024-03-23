import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
# import gym
import time
from test_generation.rl.networks import DuelingNetwork
from torch.utils.tensorboard import SummaryWriter

# 智体定义
class DQNAgent:
    def __init__(self, input_dim, action_space):
        self.q_network = DuelingNetwork(input_dim, len(action_space)).cuda()
        self.target_network = DuelingNetwork(input_dim, len(action_space)).cuda()
        self.target_network.load_state_dict(self.q_network.state_dict())
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=0.01)
        self.criterion = nn.MSELoss()

        self.action_space = action_space
        self.epsilon = 0.8
        self.epsilon_decay = 0.9
        self.epsilon_min = 0.01
        self.gamma = 0.95

    def get_action(self, state):
       
        q1, q2, q3 = self.q_network(torch.FloatTensor(state[0]).unsqueeze(0).cuda(),torch.FloatTensor(state[1]).unsqueeze(0).cuda(), torch.FloatTensor(state[2]).unsqueeze(0).cuda())
        if np.random.rand() < self.epsilon:
            a1 = np.random.choice(self.action_space)
        else:
            a1 = self.action_space[torch.argmax(q1).item()]

        if np.random.rand() < self.epsilon:
            a2 = np.random.choice(self.action_space)
        else:
            a2 = self.action_space[torch.argmax(q2).item()]

        if np.random.rand() < self.epsilon:
            a3 = np.random.choice(self.action_space)
        else:
            a3 = self.action_space[torch.argmax(q3).item()]
           
        
        return a1, a2, a3

    def train(self, experiences):
        states, actions, rewards, next_states = zip(*experiences)
        states = torch.FloatTensor(states).cuda().reshape(3,32,2)
        actions = torch.LongTensor(actions).cuda().reshape(3,32,1)
        rewards = torch.FloatTensor(rewards).cuda()
        next_states = torch.FloatTensor(next_states).cuda().reshape(3,32,2)
        # print(states)
        # 计算Q值
        q1, q2, q3 = self.q_network(states[0], states[1], states[2])
        # print(actions[0].unsqueeze(-1))
        # print(q1)
        q1_a = q1.gather(1, actions[0])
        q2_a = q2.gather(1, actions[1])
        q3_a = q3.gather(1, actions[2])

        total_q = q1_a + q2_a + q3_a

        q1_t,q2_t,q3_t = self.target_network(next_states[0], next_states[1], next_states[2])
        q1_tt = q1_t.max(1)[0]
        q2_tt = q2_t.max(1)[0]
        q3_tt = q3_t.max(1)[0]

        q_t_total = q1_tt + q2_tt + q3_tt

        target = rewards +  self.gamma * q_t_total

        loss = self.criterion(total_q, target.unsqueeze(-1))
        
        self.optimizer.zero_grad()
        loss.backward(retain_graph=True)
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def update_target(self):
        self.target_network.load_state_dict(self.q_network.state_dict())

# 训练循环
def train_marl(agents, env, episodes=100000, batch_size=32, update_every=10):

    memory = []
    for episode in range(episodes):
        # print(episode)
        state = env.reset()
        # time.sleep(3)
        episode_reward = 0
        runtime = 0
        done_n = False
        while not done_n:
            
            # time.sleep(0.5)
            runtime+=1
            # print(runtime)
            action1, action2, action3 = agents.get_action(state)
            obs_n, reward_n, done_n = env.step([action1, action2, action3], runtime)
            # print(reward_n)
            episode_reward+= reward_n
            memory.append((state, (action1, action2, action3), reward_n, obs_n))

            if len(memory) > batch_size:
                batch = random.sample(memory, batch_size)
                agents.train([(s, a, r, ns) for s, a, r, ns in batch])

            state = obs_n
        writer.add_scalar("/home/yao/Documents/landing_airsim/scripts/Reward", episode_reward, episode)
        print(episode_reward)
        if episode % update_every == 0:
            agents.update_target()


# 示例

if __name__ == '__main__':
    env = CustomEnv()
    agents = DQNAgent(2, [0,1, 2, 3, 4])
    writer = SummaryWriter(log_dir='/home/yao/Documents/landing_airsim/scripts/Reward')
    train_marl(agents, env)
    writer.close()