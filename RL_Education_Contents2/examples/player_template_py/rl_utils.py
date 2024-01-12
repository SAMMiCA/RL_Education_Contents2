#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Kyujin Choi, Taeyoung Kim
# Maintainer: Kyujin Choi (nav3549@kaist.ac.kr)

import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Participant, Game, Frame
except ImportError as err:
    print('player_deeplearning-single-ddpg: \'participant\' module cannot be imported:', err)
    raise

try:
    import _pickle as pickle
except:
    import pickle
import math
import numpy as np
import torch
import matplotlib.pyplot as plt
from torch.autograd import Variable

import helper

#reset_reason
NONE = Game.NONE
GAME_START = Game.GAME_START
SCORE_MYTEAM = Game.SCORE_MYTEAM
SCORE_OPPONENT = Game.SCORE_OPPONENT
GAME_END = Game.GAME_END
DEADLOCK = Game.DEADLOCK
GOALKICK = Game.GOALKICK
CORNERKICK = Game.CORNERKICK
PENALTYKICK = Game.PENALTYKICK
HALFTIME = Game.HALFTIME
EPISODE_END = Game.EPISODE_END

#game_state
STATE_DEFAULT = Game.STATE_DEFAULT
STATE_KICKOFF = Game.STATE_KICKOFF
STATE_GOALKICK = Game.STATE_GOALKICK
STATE_CORNERKICK = Game.STATE_CORNERKICK
STATE_PENALTYKICK = Game.STATE_PENALTYKICK

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

#robot_index
GK_INDEX = 0
D1_INDEX = 1
D2_INDEX = 2
F1_INDEX = 3
F2_INDEX = 4

USE_CUDA = torch.cuda.is_available()
FLOAT = torch.cuda.FloatTensor if USE_CUDA else torch.FloatTensor
LONG = torch.cuda.LongTensor if USE_CUDA else torch.LongTensor

def to_numpy(var):
    return var.cpu().data.numpy() if USE_CUDA else var.data.numpy()

def to_tensor(ndarray, volatile=False, requires_grad=False, dtype=FLOAT):
    return Variable(
        torch.from_numpy(ndarray), volatile=volatile, requires_grad=requires_grad
    ).type(dtype)

def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(
            target_param.data * (1.0 - tau) + param.data * tau
        )

def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(param.data)

def predict_ball_velocity(cur_ball, prev_ball, ts):
    vx = (cur_ball[X] - prev_ball[X])/ts
    vy = (cur_ball[Y] - prev_ball[Y])/ts
    vd = math.atan2(vy, vx)
    vr = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    return [vd*180/math.pi, vr]

def predict_robot_velocity(cur_posture, prev_posture, index, ts):
    vx = (cur_posture[index][X] - prev_posture[index][X])/ts
    vy = (cur_posture[index][Y] - prev_posture[index][Y])/ts
    vd = math.atan2(vy, vx)
    vr = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    return [vd*180/math.pi, vr]

def get_state(cur_posture, prev_posture, cur_posture_opp, prev_posture_opp, cur_ball, prev_ball, field, goal, max_linear_velocity):
    # relative state: (shape: 8)
    states = [[] for _ in range(5)]
    pxx = field[X] + goal[X]
    pyy = field[Y]
    for i in range(5):
        states[i] =[round((cur_ball[X] - cur_posture[i][X])/pxx, 2), round((cur_ball[Y] - cur_posture[i][Y])/pyy, 2),  
                    round((cur_ball[X] - field[X]/2)/pxx, 2), round((cur_ball[Y] - 0)/pyy, 2),
                    round((-field[X]/2 - cur_posture[i][X])/pxx, 2), round((0 - cur_posture[i][Y])/pyy, 2),
                    round(cur_posture[i][TH], 2), round((math.atan2(cur_ball[Y]-cur_posture[i][Y], cur_ball[X]-cur_posture[i][X]) - cur_posture[i][TH])/math.pi, 2)]

    return states

def get_reward(cur_posture, prev_posture, cur_ball, prev_ball, field, id):
    dist_robot2ball = helper.distance(cur_posture[id][X] , cur_ball[X], cur_posture[id][Y], cur_ball[Y])
    robot_th_error = abs(math.atan2(cur_ball[Y]-cur_posture[id][Y], cur_ball[X]-cur_posture[id][X]) - cur_posture[id][TH])
    ball_robot_dis = helper.distance(cur_posture[id][X] , cur_ball[X], cur_posture[id][Y], cur_ball[Y])
    ball_robot_dis_prev = helper.distance(prev_posture[id][X] , prev_ball[X], prev_posture[id][Y], prev_ball[Y])
    ball_robot_velocity = (ball_robot_dis_prev - ball_robot_dis)/0.05

    return (0.5*(math.exp(-1*dist_robot2ball)) + 0.5*(math.exp(-1*robot_th_error)) + np.clip(1-math.exp(-1*(ball_robot_velocity)),0,1))

class Logger():
    def __init__(self):

        self.episode = []
        self.m_episode = []
        self.value = []
        self.mean_value = []

    def update(self, episode, value, num):

        self.episode.append(episode)
        self.value.append(value)
        self.num = num
        if len(self.value) >= self.num :
            self.m_episode.append(episode - self.num/2)
            self.mean_value.append(np.mean(self.value[-self.num:]))

    def plot(self, name):
        filename = os.path.dirname(__file__) + '/TOTAL_' + str(name) + '.png'
        plt.title(str(name))
        plt.plot(self.episode, self.value, c = 'lightskyblue', label='total_reward') 
        plt.plot(self.m_episode, self.mean_value, c = 'b', label='Average_Total_Reward') 
        if len(self.episode) <= 10:
            plt.legend(loc=1)
        plt.savefig(filename)