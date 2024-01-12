#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

import random
import os
import sys

NUM_ACTIONS = 4
GO_FORWARD = 0
GO_BACKWARD = 1
TURN_RIGHT = 2
TURN_LEFT = 3

def get_action(index, max):
    ACTIONS = [[1*max, 1*max, 0, 0, 0, 1],
                [-1*max, -1*max, 0, 0, 0, 1],
                [0.1*max, -0.1*max, 0, 0, 0, 1],
                [-0.1*max, 0.1*max, 0, 0, 0, 1]]
    return ACTIONS[index]