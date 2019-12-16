import torch
import torch.autograd
import torch.optim as optim
import torch.nn as nn
from SACmodels import *
from utils import Memory

class SACagent:
    def __init__(self, state_dim, action_dim=2, hidden_dim=256, lr=1e-3, gamma=0.99, tau=1e-2, max_memory_size=1000000, action_scales=[.22, .5], maxTemp=0.5, minTemp=0.1, tempTimeScale=500000.0):
        # use cuda?
        use_cuda = torch.cuda.is_available()
        device   = torch.device("cuda" if use_cuda else "cpu")
        self.device = device
        # Params
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        self.gamma = gamma
        self.tau = tau
        self.lr = lr
        self.maxTemp = maxTemp
        self.minTemp = minTemp
        self.tempTimeScale = tempTimeScale
        self.action_scales = action_scales

        # Memory replay buffer
        self.memory = Memory(max_memory_size)

        # Initialize all networks
        self.value_net = ValueNetwork(self.state_dim, self.hidden_dim).to(device)
        self.target_value_net = ValueNetwork(self.state_dim, self.hidden_dim).to(device)
        self.soft_q_net1 = SoftQNetwork(self.state_dim, self.action_dim, self.hidden_dim).to(device)
        self.soft_q_net2 = SoftQNetwork(self.state_dim, self.action_dim, self.hidden_dim).to(device)
        self.policy_net = PolicyNetwork(self.state_dim, self.action_dim, self.hidden_dim, self.action_scales, device).to(device)

        # Copy initial parameters from value net to target value net
        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(param.data)
    

        self.value_criterion  = nn.MSELoss()
        self.soft_q_criterion1 = nn.MSELoss()
        self.soft_q_criterion2 = nn.MSELoss()

        value_lr  = self.lr #3e-4
        soft_q_lr = self.lr #3e-4
        policy_lr = self.lr #3e-4

        self.value_optimizer  = optim.Adam(self.value_net.parameters(), lr=value_lr)
        self.soft_q_optimizer1 = optim.Adam(self.soft_q_net1.parameters(), lr=soft_q_lr)
        self.soft_q_optimizer2 = optim.Adam(self.soft_q_net2.parameters(), lr=soft_q_lr)
        self.policy_optimizer = optim.Adam(self.policy_net.parameters(), lr=policy_lr)

    def get_action(self,state):
        action = self.policy_net.get_action(state)
        action = action.detach().numpy()
        return action

    def update(self,batch_size,t):
        print("STARTING NETWORK UPDATES")
        #state, action, reward, next_state, done = replay_buffer.sample(batch_size)
        state, action, reward, next_state = self.memory.sample(batch_size)

        state      = torch.FloatTensor(state).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        action     = torch.FloatTensor(action).to(self.device)
        reward     = torch.FloatTensor(reward).to(self.device)
        #done       = torch.FloatTensor(np.float32(done)).unsqueeze(1).to(device)

        predicted_q_value1 = self.soft_q_net1(state, action)
        predicted_q_value2 = self.soft_q_net2(state, action)
        predicted_value    = self.value_net(state)
        new_action, log_prob, epsilon, mean, log_std = self.policy_net.evaluate(state)
        log_prob_sum = torch.sum(log_prob, dim=1)
        joint_entropy = log_prob_sum.unsqueeze(1)
    
        # Training Q Function
        target_value = self.target_value_net(next_state)
        target_q_value = reward + self.gamma * target_value
        q_value_loss1 = self.soft_q_criterion1(predicted_q_value1, target_q_value.detach())
        q_value_loss2 = self.soft_q_criterion2(predicted_q_value2, target_q_value.detach())
        print("Q1 LOSS = " + str(q_value_loss1) + "   Q2 LOSS = " + str(q_value_loss2))

        self.soft_q_optimizer1.zero_grad()
        q_value_loss1.backward()
        self.soft_q_optimizer1.step()
        self.soft_q_optimizer2.zero_grad()
        q_value_loss2.backward()
        self.soft_q_optimizer2.step()
        # Training Value Function
        predicted_new_q_value = torch.min(self.soft_q_net1(state, new_action),self.soft_q_net2(state, new_action))
        #target_value_func = predicted_new_q_value - log_prob
        alpha = max(self.minTemp,(self.maxTemp - self.maxTemp*(t/self.tempTimeScale)))
        print("ALPHA = " + str(alpha) + " min = " + str(self.minTemp) + " max = " + str(self.maxTemp))
        target_value_func = predicted_new_q_value - alpha*joint_entropy
        value_loss = self.value_criterion(predicted_value, target_value_func.detach())
        print("VALUE LOSS = " + str(value_loss))
        self.value_optimizer.zero_grad()
        value_loss.backward()
        self.value_optimizer.step()
        # Training Policy Function
        #policy_loss = (log_prob - predicted_new_q_value).mean()
        policy_loss = (alpha*joint_entropy - predicted_new_q_value).mean()
        print("POLICY LOSS = " + str(policy_loss))
        self.policy_optimizer.zero_grad()
        policy_loss.backward()
        self.policy_optimizer.step()
    
    
        for target_param, param in zip(self.target_value_net.parameters(), self.value_net.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - self.tau) + param.data * self.tau
            )
        print("DONE WITH NETWORK UPDATES")
        #return q_value_loss1, q_value_loss2, value_loss, policy_loss
