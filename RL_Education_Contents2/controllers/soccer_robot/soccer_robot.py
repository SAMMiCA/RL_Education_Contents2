#!/usr/bin/env python3

import random
import sys

from controller import Robot

class SoccerRobot(Robot):
    def __init__(self, noise):
        Robot.__init__(self)
        self.noise = noise
        self.left_wheel = self.getMotor('left wheel motor')
        self.right_wheel = self.getMotor('right wheel motor')
        self.left_arm = self.getMotor('left arm motor')
        self.right_arm = self.getMotor('right arm motor')
        self.left_leg = self.getMotor('left leg motor')
        self.right_leg = self.getMotor('right leg motor')
        self.left_leg2 = self.getMotor('left leg motor2')
        self.right_leg2 = self.getMotor('right leg motor2')
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_arm.setPosition(float('inf'))
        self.right_arm.setPosition(float('inf'))
        self.left_leg.setPosition(float('inf'))
        self.right_leg.setPosition(float('inf'))
        self.left_leg2.setPosition(float('inf'))
        self.right_leg2.setPosition(float('inf'))

    def run(self):
        while self.step(10) != -1:
            # ignore slip noise overshoot portion
            max_speed = self.left_wheel.getMaxVelocity()
            speeds = self.getCustomData().split(' ')
            left = min(max(float(speeds[0]),  -max_speed), max_speed)
            right = min(max(float(speeds[1]), -max_speed), max_speed)
            arm1 = float(speeds[4])
            arm2 = float(speeds[5])
            leg2 = float(speeds[6])
            leg1 = float(speeds[7])
            self.left_wheel.setVelocity(left)
            self.right_wheel.setVelocity(right)
            self.left_arm.setVelocity(-arm1/1.5)
            self.right_arm.setVelocity(arm2/1.5)
            self.left_leg.setVelocity(arm1/3.5)
            self.right_leg.setVelocity(-arm2/3.5)
            self.left_leg2.setVelocity(leg1/1.5)
            self.right_leg2.setVelocity(-leg2/1.5)

    def slipNoise(self, value):
        return value * (1 + random.uniform(-self.noise, self.noise))

if len(sys.argv) > 1:
    noise = float(sys.argv[1])
else:
    noise = 0.0
soccer_robot = SoccerRobot(noise)
soccer_robot.run()
exit(0)
