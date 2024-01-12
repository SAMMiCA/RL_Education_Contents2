#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Kyujin Choi, Taeyoung Kim
# Maintainer: Kyujin Choi (nav3549@kaist.ac.kr)

import os
import sys
import math

import matplotlib.pyplot as plt

def distance(x1, x2, y1, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def degree2radian(deg):
    return deg * math.pi / 180

def radian2degree(rad):
    return rad * 180 / math.pi

def wrap_to_pi(theta):
    while (theta > math.pi):
        theta -= 2 * math.pi
    while (theta < -math.pi):
        theta += 2 * math.pi
    return theta

def set_wheel_velocity(max_linear_velocity, left_wheel, right_wheel):
    ratio_l = 1
    ratio_r = 1

    if (left_wheel > max_linear_velocity or right_wheel > max_linear_velocity):
        diff = max(left_wheel, right_wheel) - max_linear_velocity
        left_wheel -= diff
        right_wheel -= diff
    if (left_wheel < -max_linear_velocity or right_wheel < -max_linear_velocity):
        diff = min(left_wheel, right_wheel) + max_linear_velocity
        left_wheel -= diff
        right_wheel -= diff

    return left_wheel, right_wheel

def printConsole(message):
    print(message)
    sys.__stdout__.flush()

class Logger():
    def __init__(self):

        self.frame = []
        self.value = []

    def update(self, frame, value):

        self.frame.append(frame)
        self.value.append(value)

    def plot(self, name_):
        name = str(name_)
        filename = os.path.dirname(__file__) + '/' + str(name) + '.png'
        plt.title(str(name))
        plt.plot(self.frame, self.value, c = 'b', label='Average_Total_Reward')
        plt.savefig(filename)