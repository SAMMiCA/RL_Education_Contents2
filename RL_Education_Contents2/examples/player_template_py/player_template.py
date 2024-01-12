#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Participant, Game, Frame
except ImportError as err:
    print('player_template: \'participant\' module cannot be imported:', err)
    raise

import json
import math
import action
import helper
import numpy as np

from ddpg import DDPG
from helper import Logger
from parameters import Parameters

#coordinates
MY_TEAM = Frame.MY_TEAM
OP_TEAM = Frame.OP_TEAM
BALL = Frame.BALL
X = Frame.X
Y = Frame.Y
Z = Frame.Z
TH = Frame.TH
ACTIVE = Frame.ACTIVE
TOUCH = Frame.TOUCH
BALL_POSSESSION = Frame.BALL_POSSESSION

class Player(Participant):
    def init(self, info):
        params = Parameters()
        self.field = info['field']
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity'][0]
        self.frame = Frame()
        self.i = 1
        self.state_size = info['state_size']
        self.action_size = 3

        self.number_of_agents = 1

        self.previous_action = np.zeros(self.action_size)
        self.previous_state = []
        self.previous_reward = 0

        self.load = params.load
        self.play = params.play
        self.frame_skip = params.frame_skip

        self.state_with_friction = params.state_with_friction
        if self.state_with_friction:
            self.state_size += 1
        self.state_type = params.state_type
        self.action_type = params.action_type
        self.reward_type = params.reward_type
        self.trainer = DDPG(self.state_size, self.action_size, self.load, self.play)

        self.total_rewards = 0
        self.t = 0
        self.plot_reward = Logger()
        self.save_png_interval = 2500
        
    def update(self, frame):
        
        if self.i == 1:
            self.state_size = frame.state_size

        self.frame = frame
        if not self.state_with_friction:
            if (self.state_type == 'state_absolute'):
                self.state = self.frame.state_absolute
            elif (self.state_type == 'state_relative'):
                self.state = self.frame.state_relative
            elif (self.state_type == 'state_mix'):
                self.state = self.frame.state_mix
            else:
                self.state = self.frame.state_absolute
        else:
            if (self.state_type == 'state_absolute'):
                self.state = self.frame.state_absolute_wf
            elif (self.state_type == 'state_relative'):
                self.state = self.frame.state_relative_wf
            elif (self.state_type == 'state_mix'):
                self.state = self.frame.state_mix_wf
            else:
                self.state = self.frame.state_absolute_wf

        if (self.reward_type == 'reward_continuous'):
            self.reward = self.frame.reward_continuous
        elif (self.reward_type == 'reward_binary'):
            self.reward = self.frame.reward_binary
        elif (self.reward_type == 'reward_sparse'):
            self.reward = self.frame.reward_sparse
        else:
            self.reward = self.frame.reward_binary

        if not self.play:
            if self.i % self.frame_skip == 1:
                self.action = self.trainer.select_action(self.state)
            else:
                self.action = self.previous_action
        else:
            self.action = self.trainer.select_action(self.state)

        cur_posture = frame.coordinates[MY_TEAM]

        if not self.play:
        
            if self.i == 1:
                self.trainer.store_experience(self.state, self.state, self.previous_action, self.previous_reward)
            else:
                self.trainer.store_experience(self.previous_state, self.state, self.previous_action, self.previous_reward)
            
            # logging training agent's reward and plot graph
            self.t += 1
            self.total_rewards += self.reward
            if self.i % self.save_png_interval == 0:
                mean_total_reward = self.total_rewards/self.t
                self.plot_reward.update(self.i, mean_total_reward)
                self.plot_reward.plot('REWARD-GRAPH')
                # reset episode timesteps and total reward 
                self.t = 0
                self.total_rewards = 0

            # Training script: called every timestep  
            self.trainer.update()
                
            # save checkpoint
            self.trainer.save_checkpoint(self.i)
        if self.action_type == 'action_wheels':
            speeds = [self.action[0]*self.max_linear_velocity, self.action[1]*self.max_linear_velocity, 0, 0, 0, 0]
        elif self.action_type == 'action_position':
            speeds = action.go_to(self.action[0]*self.field[0]/2, self.action[1]*self.field[1]/2, self.action[2], cur_posture[0], self.max_linear_velocity)
        else:
            speeds = [self.action[0]*self.max_linear_velocity, self.action[1]*self.max_linear_velocity, 0, 0, 0, 0]

        print('Time:  ', round((self.i-1)*0.05,2))
        print('State: ', self.state)
        print('Action:', self.action)
        print('Reward:', self.reward)
        helper.printConsole('---------')
        self.set_speeds(speeds)

        self.i += 1

        self.previous_state = self.state
        self.previous_action = self.action
        self.previous_reward = self.reward

if __name__ == '__main__':
    player = Player()
    player.run()