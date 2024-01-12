#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Kyujin Choi, Taeyoung Kim
# Maintainer: Kyujin Choi (nav3549@kaist.ac.kr)

import os
import json

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

from parameters import Parameters

def fanin_init(size, fanin=None):
    fanin = fanin or size[0]
    v = 1. / np.sqrt(fanin)
    return torch.Tensor(size).uniform_(-v, v)

class Actor(nn.Module):
    def __init__(self, nb_states, nb_actions, init_w=3e-3):
        super(Actor, self).__init__()
        params = Parameters()
        self.num_layers = params.num_layers
        self.hidden_dim = params.hidden_dim
        self.input_linear = nn.Linear(nb_states, self.hidden_dim)
        self.middle_linear = nn.Linear(self.hidden_dim, self.hidden_dim)
        self.output_linear = nn.Linear(self.hidden_dim, nb_actions)
        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()

        self.init_weights(init_w)
    
    def init_weights(self, init_w):
        self.input_linear.weight.data = fanin_init(self.input_linear.weight.data.size())
        self.middle_linear.weight.data = fanin_init(self.middle_linear.weight.data.size())
        self.output_linear.weight.data.uniform_(-init_w, init_w)

    def forward(self, x):
        out = self.input_linear(x)
        out = self.relu(out)
        for _ in range(0, self.num_layers):
            out = self.middle_linear(out)
            out = self.relu(out)
        out = self.output_linear(out)
        out = self.tanh(out)
        return out

class Critic(nn.Module):
    def __init__(self, nb_states, nb_actions, init_w=3e-3):
        super(Critic, self).__init__()
        params = Parameters()
        self.num_layers = params.num_layers
        self.hidden_dim = params.hidden_dim
        self.input_linear = nn.Linear(nb_states, self.hidden_dim)
        self.action_linear = nn.Linear(self.hidden_dim+nb_actions, self.hidden_dim)
        self.middle_linear = nn.Linear(self.hidden_dim, self.hidden_dim)
        self.output_linear = nn.Linear(self.hidden_dim, 1)
        self.relu = nn.ReLU()

        self.init_weights(init_w)
    
    def init_weights(self, init_w):
        self.input_linear.weight.data = fanin_init(self.input_linear.weight.data.size())
        self.action_linear.weight.data = fanin_init(self.action_linear.weight.data.size())
        self.middle_linear.weight.data = fanin_init(self.middle_linear.weight.data.size())
        self.output_linear.weight.data.uniform_(-init_w, init_w)

    def forward(self, xs):
        x, a = xs
        out = self.input_linear(x)
        out = self.relu(out)
        out = self.action_linear(torch.cat([out, a],1))
        out = self.relu(out)
        if self.num_layers > 1:
            for _ in range(1, self.num_layers):
                out = self.middle_linear(out)
                out = self.relu(out)
        out = self.output_linear(out)
        return out