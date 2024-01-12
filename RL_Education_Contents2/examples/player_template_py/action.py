#!/usr/bin/env python3

# Author(s): Taeyoung Kim, Chansol Hong, Luiz Felipe Vecchietti
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../common')
try:
    from participant import Game, Frame
except ImportError as err:
    print('player_rulebasedC: \'participant\' module cannot be imported:', err)
    raise

import math
import helper
import random
import os
import sys

NUM_ACTIONS = 6
GO_FORWARD = 0
GO_BACKWARD = 1
TURN_RIGHT = 2
TURN_LEFT = 3

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

def get_action(index, max):
    ACTIONS = [[1*max, 1*max, 0, 0, 0, 0],
                [-1*max, -1*max, 0, 0, 0, 0],
                [0.1*max, -0.1*max, 0, 0, 0, 0],
                [-0.1*max, 0.1*max, 0, 0, 0, 0],
                [0, 0, 10, 5, 0, 0],
                [0, 0, 0, 0, 0, 0]]
    return ACTIONS[index]

def go_to(x, y, turn_control, cur_posture, max_linear_velocity):
    sign = 1
    
    if cur_posture[BALL_POSSESSION]:
        if turn_control < 0:
            kd = 0
            ka = 0.3
        else:
            kd = 5
            ka = 0.3
    else:
        kd = 5
        ka = 0.3      

    dx = x - cur_posture[X]
    dy = y - cur_posture[Y]
    d_e = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    desired_th = math.atan2(dy, dx)

    d_th = helper.wrap_to_pi(desired_th - cur_posture[TH])
    
    if not cur_posture[BALL_POSSESSION]:
        if (d_th > helper.degree2radian(90)):
            d_th -= math.pi
            sign = -1
        elif (d_th < helper.degree2radian(-90)):
            d_th += math.pi
            sign = -1

    left_wheel, right_wheel = helper.set_wheel_velocity(max_linear_velocity,
                sign * (kd * d_e - ka * d_th), 
                sign * (kd * d_e + ka * d_th))

    return [left_wheel, right_wheel, 0, 0, 0, 0]