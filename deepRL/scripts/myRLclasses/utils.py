import numpy as np
#import gym
from collections import deque
import random

# Ornstein-Ulhenbeck Process
# Taken from #https://github.com/vitchyr/rlkit/blob/master/rlkit/exploration_strategies/ou_strategy.py
class OUNoise(object):
    def __init__(self, mu=0.0, theta=0.15, max_sigma=0.3, min_sigma=0.2, decay_period=500000.0,action_mins=[-.22, -.5], action_maxs=[.22, .5]):
        self.mu           = mu
        self.theta        = theta
        self.sigma        = max_sigma
        self.max_sigma    = max_sigma
        self.min_sigma    = min_sigma
        self.decay_period = decay_period
        self.action_dim   = 2 #action_space.shape[0]
        self.action_mins = action_mins
        self.action_maxs = action_maxs
        self.low          = -.5
        self.high         = .5
        self.reset()
        
    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu
        
    def evolve_state(self):
        x  = self.state
        dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(self.action_dim)
        self.state = x + dx
        return self.state
    
    def get_action(self, action, t=0):
        ou_state = self.evolve_state()
        #print("OUstate: " + str(ou_state))
        self.sigma = self.max_sigma - (self.max_sigma - self.min_sigma) * min(1.0, t / self.decay_period)
        noisy_action = action + ou_state
        noisy_action[0] = np.clip(noisy_action[0],self.action_mins[0],self.action_maxs[0])
        noisy_action[1] = np.clip(noisy_action[1],self.action_mins[1],self.action_maxs[1])

        #return np.clip(action + ou_state, self.low, self.high)
        return noisy_action


# https://github.com/openai/gym/blob/master/gym/core.py
"""
class NormalizedEnv(gym.ActionWrapper):
    # Wrap action

    def action(self, action):
        act_k = (self.action_space.high - self.action_space.low)/ 2.
        act_b = (self.action_space.high + self.action_space.low)/ 2.
        return act_k * action + act_b

    def reverse_action(self, action):
        act_k_inv = 2./(self.action_space.high - self.action_space.low)
        act_b = (self.action_space.high + self.action_space.low)/ 2.
        return act_k_inv * (action - act_b)

"""

class Memory:
    def __init__(self, max_size):
        self.max_size = max_size
        self.buffer = deque(maxlen=max_size)

    def push(self, state, action, reward, next_state):
        experience = (state, action, np.array([reward]), next_state)
        self.buffer.append(experience)

    def sample(self, batch_size):
        state_batch = []
        action_batch = []
        reward_batch = []
        next_state_batch = []

        batch = random.sample(self.buffer, batch_size)

        for experience in batch:
            state, action, reward, next_state = experience
            state_batch.append(state)
            action_batch.append(action)
            reward_batch.append(reward)
            next_state_batch.append(next_state)

        return state_batch, action_batch, reward_batch, next_state_batch

    def __len__(self):
        return len(self.buffer)
