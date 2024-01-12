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
    print('player_random-walk: \'participant\' module cannot be imported:', err)
    raise

import action

class RandomWalkPlayer(Participant):
    def init(self, info):
        self.number_of_robots = info['number_of_robots']
        self.max_linear_velocity = info['max_linear_velocity'][0]
        self.frame = Frame()
        self.i = 0

    def update(self, frame):

        self.frame = frame
        
        speeds = []
        for i in range(self.number_of_robots):
            index = random.randrange(action.NUM_ACTIONS)
            speeds.extend(action.get_action(index, self.max_linear_velocity))
        self.set_speeds(speeds)

        self.i += 1

if __name__ == '__main__':
    player = RandomWalkPlayer()
    player.run()
