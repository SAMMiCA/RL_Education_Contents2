import os
import random
import json

from networks import Actor, Critic
import torch
import torch.nn as nn
import torch.nn.functional as F
from copy import deepcopy
import torch.optim as optim
import numpy as np

import helper
from episode_memory import Memory
from rl_utils import to_numpy, to_tensor, soft_update, hard_update
from parameters import Parameters

CHECKPOINT = os.path.join(os.path.dirname(__file__), 'models')

class DDPG(object):
    def __init__(self, nb_states, nb_actions, load=False, play=False):
        params = Parameters()
        self.nb_states = nb_states
        self.nb_actions = nb_actions
        self.load = load
        self.play = play

        self._iterations = 0
        self.update_steps = 200
        self.observation_steps = 300
        self.save_num = 5000

        # Hyper-parameters
        self.batch_size = params.ddpg_batch_size
        self.tau = 0.001
        self.gamma = params.ddpg_gamma
        self.epsilon = 1.0
        self.depsilon = 1.0/params.ddpg_dec_exploration
        actor_lr = params.ddpg_actor_lr
        critic_lr = params.ddpg_critic_lr
        self.grad_norm_clip = params.ddpg_grad_norm_clip

        self.actor = Actor(self.nb_states, self.nb_actions)
        self.actor_target = Actor(self.nb_states, self.nb_actions)
        self.actor_optim  = optim.Adam(self.actor.parameters(), lr=actor_lr)

        self.critic = Critic(self.nb_states, self.nb_actions)
        self.critic_target = Critic(self.nb_states, self.nb_actions)
        self.critic_optim  = optim.Adam(self.critic.parameters(), lr=critic_lr)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if self.load:
            self.actor.load_state_dict(
                torch.load(os.path.join(CHECKPOINT, "actor.pkl"), map_location=torch.device(self.device)))
            self.critic.load_state_dict(
                torch.load(os.path.join(CHECKPOINT, "critic.pkl"), map_location=torch.device(self.device)))
            helper.printConsole("loading variables...")

        hard_update(self.actor_target, self.actor) # Make sure target is with the same weight
        hard_update(self.critic_target, self.critic)
        
        if self.play:
            self.eval()
        else:
            self.train()
        self.to_device()

        #Create replay buffer
        self.memory = Memory(params.ddpg_buffer_size)

    def update_policy(self):
        # Sample batch
        batch = self.memory.sample(self.batch_size)
        
        state_batch = torch.Tensor(batch.state).squeeze().to(self.device)
        next_state_batch = torch.Tensor(batch.next_state).squeeze().to(self.device)
        action_batch = torch.Tensor(batch.action).to(self.device)
        reward_batch = torch.Tensor(batch.reward).unsqueeze(1).to(self.device)

        # Prepare for the target q batch
        next_q_values = self.critic_target([
            next_state_batch,
            self.actor_target(next_state_batch),
        ])

        target_q_batch = reward_batch + \
            self.gamma*next_q_values

        # Critic update
        self.critic.zero_grad()

        q_batch = self.critic([state_batch, action_batch])

        value_loss = nn.MSELoss()(q_batch, target_q_batch)
        value_loss.backward()
        grad_norm_critic = torch.nn.utils.clip_grad_norm_(self.critic.parameters(), self.grad_norm_clip)
        self.critic_optim.step()

        # Actor update
        self.actor.zero_grad()

        policy_loss = -self.critic([
            state_batch,
            self.actor(state_batch)
        ])

        policy_loss = policy_loss.mean()
        policy_loss.backward()
        grad_norm_actor = torch.nn.utils.clip_grad_norm_(self.actor.parameters(), self.grad_norm_clip)
        self.actor_optim.step()

        # Target update
        soft_update(self.actor_target, self.actor, self.tau)
        soft_update(self.critic_target, self.critic, self.tau)
        return value_loss.cpu().data.numpy(), policy_loss.cpu().data.numpy()

    def train(self):
        self.actor.train()
        self.actor_target.train()
        self.critic.train()
        self.critic_target.train()

    def eval(self):
        self.actor.eval()
        self.actor_target.eval()
        self.critic.eval()
        self.critic_target.eval()

    def to_device(self):
        self.actor.to(self.device)
        self.actor_target.to(self.device)
        self.critic.to(self.device)
        self.critic_target.to(self.device)

    def store_experience(self, state, next_state, act, rew):
        self.memory.push(state, next_state, act, rew)

    def select_action(self, s_t):
        action = to_numpy(
            self.actor(to_tensor(np.array([s_t])))
        ).squeeze(0)

        if not self.play:
            if np.random.rand() <= self.epsilon:
                for i in range(self.nb_actions):
                    action[i] = action[i] + np.random.normal(loc=0.0, scale=0.3)

        if not self.play:
            self.epsilon -= self.depsilon

        return action

    def print_loss(self, vloss, ploss, iteration):
        if self._iterations % 100 == 0: # Print information every 100 iterations
            helper.printConsole("======================================================")
            helper.printConsole("Epsilon: " + str(self.epsilon))
            helper.printConsole("iterations: " + str(self._iterations))
            helper.printConsole("Value Loss: " + str(vloss))
            helper.printConsole("Policy Loss: " + str(ploss))
            helper.printConsole("======================================================")

    def save_checkpoint(self, iteration):
        if iteration % self.save_num == 0:
            torch.save(
                self.actor.state_dict(),
                '{}/actor.pkl'.format(CHECKPOINT)
            )
            torch.save(
                self.critic.state_dict(),
                '{}/critic.pkl'.format(CHECKPOINT)
            )
            helper.printConsole("Saved Checkpoint.")

    def update(self):
        if len(self.memory) > self.observation_steps:
            self._iterations += 1
            vloss, ploss = self.update_policy()
            self.print_loss(vloss, ploss, self._iterations)