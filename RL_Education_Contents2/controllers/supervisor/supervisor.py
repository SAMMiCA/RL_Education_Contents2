#!/usr/bin/env python3

import errno
import json
import math
import os
import random
import select
import socket
import string
import subprocess
import sys
import time
import collections
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '../submodules'))

from controller import Supervisor

from player import Game

import constants

def random_string(length):
    """Generate a random string with the combination of lowercase and uppercase letters."""
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(length))

def get_key(rpc):
    """The key is the first argument of the RPC."""
    first = rpc.find('"') + 1
    return rpc[first:rpc.find('"', first)]

def get_robot_name(self, color, id):
    name = self.constants.DEF_ROBOT_PREFIX
    if color == self.constants.TEAM_RED:
        name += 'R'
    elif color == self.constants.TEAM_BLUE:
        name += 'B'
    else:
        sys.stderr.write("Error: get_robot_name: Invalid team color.\n")
    name += str(id)
    return name

def get_role_name(self, role):
    if role == self.constants.TEAM_RED:
        return 'team red'
    if role == self.constants.TEAM_BLUE:
        return 'team blue'
    sys.stderr.write("Error: get_role_name: Invalid role.\n")
    return ''

def next_available_port(server_port, max_instances):
    port = server_port
    while True:
        if port > server_port + max_instances:
            print('Maximum number of instances reached')
            return port
        found = False
        # test if the port is available
        testSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            testSocket.bind(('0.0.0.0', port))
            found = True
        except socket.error as e:
            found = False
        finally:
            testSocket.close()
            if found:
                return port
            port += 1

class TcpServer:
    def __init__(self, host, server_port, max_instances):
        self.port = next_available_port(server_port, max_instances)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.setblocking(False)
        self.server.bind((host, self.port))
        self.server.listen(5)
        self.connections = [self.server]
        self.unprocessedData = {}
        self.unprocessedData[self.server.fileno()] = ''

    def get_port(self):
        return self.port

    def send_to_all(self, message):  # send message to all clients
        for client in self.connections:
            if client == self.server:
                continue
            self.send(client, message)

    def send(self, client, message):  # send message to a single client
        if client.fileno() == -1:  # was closed
            return
        try:
            client.sendall(message.encode())
        except ConnectionAbortedError:
            self.connections.remove(client)

    def spin(self, game_supervisor):  # handle asynchronous requests from clients
        def cleanup(s):
            print('Cleanup')
            if s in self.connections:
                self.connections.remove(s)
            s.close()
        while True:
            readable, writable, exceptional = select.select(self.connections, [], self.connections, 0)
            if not readable and not writable and not exceptional:
                return
            for s in readable:
                if s is self.server:
                    connection, client_address = s.accept()
                    connection.setblocking(False)
                    self.connections.append(connection)
                    self.unprocessedData[connection.fileno()] = ''
                    print('Accepted ', client_address)
                else:
                    success = True
                    data = ''
                    try:
                        while True:
                            d = s.recv(4096)
                            if not d:
                                break
                            data += d.decode()
                    except socket.error as e:
                        if e.args[0] == errno.EWOULDBLOCK:
                            success = True
                        else:
                            if e.args[0] != 10053:  # WSAECONNABORTED
                                print('Error caught: ', e.args[0])
                            success = False
                    if data and success:
                        self.unprocessedData[s.fileno()] = \
                            game_supervisor.callback(s, self.unprocessedData[s.fileno()] + data)
                    else:
                        print('Closing')
                        cleanup(s)
            for s in exceptional:
                print('Exceptional')
                cleanup(s)

class GameSupervisor(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.constants = constants
        self.basicTimeStep = int(self.getBasicTimeStep())
        self.timeStep = self.constants.PERIOD_MS
        self.waitReady = 0
        self.sync= [False, False]

        self.speeds_buffer = [[0 for _ in range(30)],[0 for _ in range(30)]]

        self.pre_pos = [[[0 for _ in range(5)] for _ in range(self.constants.NUMBER_OF_ROBOTS)] for _ in range(2)]
        self.pre_th = [0, 0]
        self.pre_ball = [0, 0, 0]

        # arm and leg
        self.arm_count = [[0,0,0,0,0],[0,0,0,0,0]]
        self.p_count = [[1,1,1,1,1],[1,1,1,1,1]]
        self.arm_speed = [[0,0,0,0,0],[0,0,0,0,0]]
        # ball_possession
        self.ball_possession = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        # spotlight
        self.spotlight = self.getFromDef("DEF_BALLSPOTLIGHT")

        self.receiver = self.getReceiver(self.constants.NAME_RECV)
        self.receiver.enable(self.constants.RECV_PERIOD_MS)

        self.viewpointNode = self.getFromDef(self.constants.DEF_AUDVIEW)
        # DEF_GRASS is not visible to cam a and cam b, optional
        grass = self.getFromDef(self.constants.DEF_GRASS)
        # BALLSHAPE is visible only to viewpoint, ORANGESHAPE is to cam_a and cam_b, mandatory
        ball = self.getFromDef(self.constants.DEF_BALLSHAPE)
        # Stadium is visible only to viewpoint, optional
        stadium = self.getFromDef(self.constants.DEF_STADIUM)
        # Wall is visible only to robots
        wall = self.getFromDef(self.constants.DEF_WALL)
        if wall:
            wall.setVisibility(self.viewpointNode, False)
        # VisualWall is visible only to viewpoint, optional
        visual_wall = self.getFromDef(self.constants.DEF_VISWALL)
        # patches'
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                robot = self.getFromDef(get_robot_name(self, team, id))
                patches = robot.getField('patches')
                number = patches.getMFNode(0)

    ## basic ##
    def step(self, timeStep, runTimer=False):
        for i in range(0, timeStep, self.basicTimeStep):
            if Supervisor.step(self, self.basicTimeStep) == -1:
                return -1
            if runTimer:
                self.time += self.basicTimeStep
            self.update_label()

    def get_role(self, rpc):
        key = get_key(rpc)
        for role in self.constants.ROLES:
            if role in self.role_info and 'key' in self.role_info[role] and self.role_info[role]['key'] == key:
                return role
        sys.stderr.write("Error: get_role: invalid key.\n")
        return -1

    def callback(self, client, message):
        unprocessed = ''
        if not message.startswith('aiwc.'):
            print('Error, AIWC RPC messages should start with "aiwc.".')
            return unprocessed

        # Handle concatenated messages
        data = message.split('aiwc.')
        for command in data:
            if not command:
                continue
            if not command.endswith(')'):
                unprocessed += 'aiwc.' + command
                continue
            role = self.get_role(command)
            self.role_client[role] = client
            if command.startswith('get_info('):
                print('Server receive aiwc.get_info from ' + get_role_name(self, role))
                if role == self.constants.TEAM_RED:
                    self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_RED]))
                elif role == self.constants.TEAM_BLUE:
                    self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_BLUE]))
                # else:
                #     self.tcp_server.send(client, json.dumps(self.role_info[self.constants.TEAM_RED]))
            elif command.startswith('ready('):
                self.ready[role] = True
                print('Server receive aiwc.ready from ' + get_role_name(self, role))
            elif command.startswith('set_speeds('):
                start = command.find('",') + 2
                end = command.find(')', start)
                speeds = command[start:end]
                speeds = [float(i) for i in speeds.split(',')]
                self.speeds_buffer[role] = speeds
                self.sync[role] = True
                self.set_speeds(role, speeds)
            else:
                print('Server received unknown message', message)
        return unprocessed

    def get_state(self, team, id, type='mix'):
        ball = self.ball_position
        pos = self.get_robot_posture(team, id)
        dx = ball[0] - pos[0]
        dy = ball[1] - pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        desired_th = math.atan2(dy, dx)
        if team == self.constants.TEAM_RED:
            th = pos[3]
        else:
            th = pos[3] + self.constants.PI if pos[3] < 0 else pos[3] - self.constants.PI
        d_th = desired_th - th
        if type == 'mix':
            state = [round(2*pos[0]/self.constants.FIELD_LENGTH, 2),
                     round(2*pos[1]/self.constants.FIELD_WIDTH, 2),
                     round(th/self.constants.PI, 2),
                     round(distance/self.constants.FIELD_LENGTH, 2),  
                     round(d_th/self.constants.PI, 2)]
        elif type == 'absolute':
            state = [round(2*pos[0]/self.constants.FIELD_LENGTH, 2),
                     round(2*pos[1]/self.constants.FIELD_WIDTH, 2),
                     round(th/self.constants.PI, 2),
                     round(2*ball[0]/self.constants.FIELD_LENGTH, 2),
                     round(2*ball[1]/self.constants.FIELD_WIDTH, 2)]
        elif type == 'relative':
            state = [round(2*dx/self.constants.FIELD_LENGTH, 2),
                     round(2*pos[1]/self.constants.FIELD_WIDTH, 2),
                     round(th/self.constants.PI, 2),
                     round(distance/self.constants.FIELD_LENGTH, 2),  
                     round(d_th/self.constants.PI, 2)]
        return state

    def get_reward(self, team, id, type='continuous'):
        # distance robot 2 ball
        ball = self.ball_position
        pos = self.get_robot_posture(team, id)
        dx = ball[0] - pos[0]
        dy = ball[1] - pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        ball_possession = self.robot[team][id]['ball_possession']
        dx_g = self.constants.FIELD_LENGTH/2 - pos[0]
        dy_g = 0 - pos[1]
        desired_th = math.atan2(dy_g, dx_g)
        if team == self.constants.TEAM_RED:
            th = pos[3]
        else:
            th = pos[3] + self.constants.PI if pos[3] < 0 else pos[3] - self.constants.PI
        d_th = desired_th - th
        reward = 0
        if type == 'continuous':
            reward = math.exp(-1*distance) - 1
        elif type == 'binary':
            if not ball_possession:
                dx = ball[0] - self.pre_pos[team][id][0]
                dy = ball[1] - self.pre_pos[team][id][1]
                pre_distance = math.sqrt(dx*dx + dy*dy)
                if pre_distance - distance > 0.01:
                    reward = 0
                else:
                    reward = -1
            else:
                if team == self.constants.TEAM_RED:
                    th = self.pre_th[team]
                else:
                    th = self.pre_th[team] + self.constants.PI if self.pre_th[team] < 0 else self.pre_th[team] - self.constants.PI
                pre_d_th = desired_th - th
                if abs(pre_d_th) - abs(d_th) > 0.01:
                    reward = 0
                else:
                    reward = -1
        elif type == 'sparse':
            if distance < 0.1:
                reward = 0
            else:
                reward = -1
        return reward

    def publish_current_frame(self, reset_reason=None):
        frame_team_red = self.generate_frame(self.constants.TEAM_RED, reset_reason)  # frame also sent to commentator and reporter
        frame_team_blue = self.generate_frame(self.constants.TEAM_BLUE, reset_reason)
        for role in self.constants.ROLES:
            if role in self.role_client:
                frame = frame_team_blue if role == self.constants.TEAM_BLUE else frame_team_red
                self.tcp_server.send(self.role_client[role], json.dumps(frame))

    def generate_frame(self, team, reset_reason=None):
        opponent = self.constants.TEAM_BLUE if team == self.constants.TEAM_RED else self.constants.TEAM_RED
        frame = {}
        frame['time'] = self.getTime()
        frame['coordinates'] = [None] * 3
        for t in self.constants.TEAMS:
            frame['coordinates'][t] = [None] * self.constants.NUMBER_OF_ROBOTS
            c = team if t == self.constants.TEAM_RED else opponent
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                frame['coordinates'][t][id] = [None] * 7
                pos = self.get_robot_posture(c, id)
                frame['coordinates'][t][id][0] = pos[0] if team == self.constants.TEAM_RED else -pos[0]
                frame['coordinates'][t][id][1] = pos[1] if team == self.constants.TEAM_RED else -pos[1]
                frame['coordinates'][t][id][2] = pos[2]
                if team == self.constants.TEAM_RED:
                    frame['coordinates'][t][id][3] = pos[3]
                else:
                    frame['coordinates'][t][id][3] = pos[3] + self.constants.PI if pos[3] < 0 else pos[3] - self.constants.PI
                frame['coordinates'][t][id][4] = self.robot[c][id]['active']
                frame['coordinates'][t][id][5] = self.robot[c][id]['touch']
                frame['coordinates'][t][id][6] = self.robot[c][id]['ball_possession']
        frame['coordinates'][2] = [None] * 3
        frame['coordinates'][2][0] = self.ball_position[0] if team == self.constants.TEAM_RED else -self.ball_position[0]
        frame['coordinates'][2][2] = self.ball_position[2]
        frame['coordinates'][2][1] = self.ball_position[1] if team == self.constants.TEAM_RED else -self.ball_position[1]
        
        frame['state_absolute'] = [None] * 5
        frame['state_relative'] = [None] * 5
        frame['state_mix'] = [None] * 5

        frame['state_absolute_wf'] = [None] * 6
        frame['state_relative_wf'] = [None] * 6
        frame['state_mix_wf'] = [None] * 6
        
        frame['state_size'] = len(frame['state_mix'])
        frame['reward_continuous'] = [None] * 1
        frame['reward_binary'] = [None] * 1
        frame['reward_sparse'] = [None] * 1
        # filling the state
        frame['friction'] = self.current_friction

        frame['state_absolute'] = self.get_state(team, 0, "absolute")
        frame['state_relative'] = self.get_state(team, 0, "relative")
        frame['state_mix'] = self.get_state(team, 0, "mix")

        frcition_state = max(min(2.5*(self.current_friction-0.5),1),-1)
        frame['state_absolute_wf'] = self.get_state(team, 0, "absolute") + [frcition_state]
        frame['state_relative_wf'] = self.get_state(team, 0, "relative") + [frcition_state]
        frame['state_mix_wf'] = self.get_state(team, 0, "mix") + [frcition_state]
        
        frame['reward_continuous'] = self.get_reward(team, 0, 'continuous')
        frame['reward_binary'] = self.get_reward(team, 0, 'binary')
        frame['reward_sparse'] = self.get_reward(team, 0, 'sparse')
        self.pre_pos[team][0] = self.get_robot_posture(team, 0)
        self.pre_th[team] = self.get_robot_posture(team, 0)[3]
        self.pre_ball = self.ball_position
        return frame

    def robot_in_field(self, team, id):
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        if abs(y) < self.constants.GOAL_WIDTH / 2:
            if abs(x) > self.constants.FIELD_LENGTH / 2 + self.constants.GOAL_DEPTH:
                return False
            else:
                return True
        if abs(x) > self.constants.FIELD_LENGTH / 2:
            return False
        else:
            return True

    def ball_in_field(self):
        pos = self.get_ball_position()
        # checking with absolute values is sufficient since the field is symmetrical
        abs_x = abs(pos[0])
        abs_y = abs(pos[1])
        abs_z = abs(pos[2])

        if (abs_x > self.constants.FIELD_LENGTH / 2) and \
           (abs_y < self.constants.GOAL_WIDTH / 2) and \
           (abs_z >= 0.5):
            return False
        if (abs_x > self.constants.FIELD_LENGTH / 2 + self.constants.WALL_THICKNESS) and \
           (abs_y > self.constants.GOAL_WIDTH / 2 + self.constants.WALL_THICKNESS):
            return False
        if abs_y > self.constants.FIELD_WIDTH / 2 + self.constants.WALL_THICKNESS:
            return False
        # check triangular region at the corner
        cs_x = self.constants.FIELD_LENGTH / 2 - self.constants.CORNER_LENGTH
        cs_y = self.constants.FIELD_WIDTH / 2 + self.constants.WALL_THICKNESS
        ce_x = self.constants.FIELD_LENGTH / 2 + self.constants.WALL_THICKNESS
        ce_y = self.constants.FIELD_WIDTH / 2 - self.constants.CORNER_LENGTH
        if cs_x < abs_x and abs_x < ce_x:
            border_y = ce_y + (abs_x - ce_x) * (ce_y - cs_y) / (ce_x - cs_x)
            if abs_y > border_y:
                return False
        return True

    ## get informations ##
    def get_robot_touch_ball(self):
        rc = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        while self.receiver.getQueueLength() > 0:
            message = self.receiver.getData()
            for team in self.constants.TEAMS:
                for id in range(self.constants.NUMBER_OF_ROBOTS):
                    cond = self.speeds_buffer[team][6*id+2] > 0
                    if message[2 * id + team] == 1:
                        rc[team][id] = True
            self.receiver.nextPacket()
        return rc

    def flush_touch_ball(self):
        while self.receiver.getQueueLength() > 0:
            self.receiver.nextPacket()

    def get_robot_posture(self, team, id):
        position = self.robot[team][id]['node'].getPosition()
        orientation = self.robot[team][id]['node'].getOrientation()
        f = 1
        x = position[0]
        y = -position[2]
        z = position[1]
        th = (0) + math.atan2(orientation[2], orientation[8]) + self.constants.PI / 2
        # Squeeze the orientation range to [-PI, PI]
        while th > self.constants.PI:
            th -= 2 * self.constants.PI
        while th < -self.constants.PI:
            th += 2 * self.constants.PI
        stand = orientation[4] > 0.8
        return [f * x, f * y, z, th, stand]

    def get_ball_position(self):
        f = 1
        position = self.ball.getPosition()
        x = position[0]
        y = -position[2]
        z = position[1]
        return [f * x, f * y, z]

    def reset_ball(self, x, z):
        f = 1
        self.ball.getField('translation').setSFVec3f([f * x, 1.5 * self.constants.BALL_RADIUS, -f * z])
        self.ball.getField('rotation').setSFRotation([0, 1, 0, 0])
        self.ball.resetPhysics()

    def reset_robot(self, team, id, x, y, z, th):
        robot = self.getFromDef(get_robot_name(self, team, id))
        f = 1
        translation = [f * x, y, f * -z]
        rotation = [0, 1, 0, th + (0)]

        al = robot.getField('axleLength').getSFFloat()
        h = robot.getField('height').getSFFloat()
        wr = robot.getField('wheelRadius').getSFFloat()

        lwTranslation = [-al / 2, (-h + 2 * wr) / 2, 0]
        rwTranslation = [al / 2, (-h + 2 * wr) / 2, 0]
        wheelRotation = [1, 0, 0, self.constants.PI / 2]

        robot.getField('translation').setSFVec3f(translation)
        robot.getField('rotation').setSFRotation(rotation)
        robot.getField('lwTranslation').setSFVec3f(lwTranslation)
        robot.getField('lwRotation').setSFRotation(wheelRotation)
        robot.getField('rwTranslation').setSFVec3f(rwTranslation)
        robot.getField('rwRotation').setSFRotation(wheelRotation)
        self.relocate_all(team, id)

        robot.resetPhysics()
        self.robot[team][id]['active'] = True
        self.robot[team][id]['touch'] = False
        self.robot[team][id]['ball_possession'] = False
        self.robot[team][id]['fall_time'] = self.time
        self.robot[team][id]['sentout_time'] = 0
        self.stop_robots()

    def reset(self, red_formation, blue_formation):
        # reset the ball
        if red_formation == self.constants.FORMATION_DEFAULT or red_formation == self.constants.FORMATION_KICKOFF:
            self.reset_ball(self.constants.BALL_POSTURE[self.constants.BALL_DEFAULT][0],
                            self.constants.BALL_POSTURE[self.constants.BALL_DEFAULT][1])

        # reset the robots
        for team in self.constants.TEAMS:
            if team == self.constants.TEAM_RED:
                s = 1
                a = 0
                formation = red_formation
            else:
                s = -1
                a = self.constants.PI
                formation = blue_formation
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.reset_robot(team, id,
                                 self.constants.ROBOT_FORMATION[formation][id][0] * s,
                                 0.09 / 2,
                                 self.constants.ROBOT_FORMATION[formation][id][1] * s,
                                 self.constants.ROBOT_FORMATION[formation][id][2] + a - self.constants.PI / 2)

        # reset recent touch
        self.recent_touch = [[False] * self.constants.NUMBER_OF_ROBOTS, [False] * self.constants.NUMBER_OF_ROBOTS]
        self.deadlock_time = self.time
        # flush touch packet
        self.flush_touch_ball()
        # reset slider, arm and leg
        self.relocate_all_every()
        self.touch_flag_corner = False
        self.touch_flag_penalty = False

    def lock_all_robots(self, locked):
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.robot[t][id]['active'] = not locked
        if locked:
            self.relocate_all_every()

    def stop_robots(self):
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.arm_count[t][id] = 0
                self.arm_speed[t][id] = 0
            self.set_speeds(t, [0, 0, 0, 0, 0, 0, 0, 0])

    ## label ##
    def update_label(self):
        seconds = self.time / 1000.0
        minutes = seconds // 60
        seconds -= minutes * 60
        self.setLabel(0, '%d:%05.2f (1st half)' % (minutes, seconds), 0.01, 0.01, 0.11, 0xe6b800, 0, 'Impact')
        self.setLabel(4, 'Friction: %05.2f' % (self.current_friction), 0.4, 0.01, 0.1, 0xff0000, 0, 'Impact')


    def set_wheel_velocity(self, max_linear_velocity, left_wheel, right_wheel):
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

    ## speed ##
    def set_speeds(self, team, speeds):
        letter = 'R' if team == self.constants.TEAM_RED else 'B'
        def_robot_prefix = self.constants.DEF_ROBOT_PREFIX + letter
        for id in range(self.constants.NUMBER_OF_ROBOTS):
            robot = self.getFromDef(def_robot_prefix + str(id))
            if self.robot[team][id]['active']:
                fs_speed = 0
                bs_speed = 0
                max_speed = 0
                left_wheel_speed = 0
                right_wheel_speed = 0
                max_speed = self.constants.MAX_LINEAR_VELOCITY[id]
                left_wheel_speed, right_wheel_speed = self.set_wheel_velocity(max_speed, speeds[id * 6], speeds[id * 6 + 1])
                left_wheel_speed = left_wheel_speed / self.constants.WHEEL_RADIUS[id]
                right_wheel_speed = right_wheel_speed / self.constants.WHEEL_RADIUS[id]
                if self.robot[team][id]['node'].getOrientation()[4] < 0.9:
                    left_wheel_speed, right_wheel_speed = 0, 0
                arm_leg_speed = 0,0,0,0
                robot.getField('customData').setSFString(
                    "%f %f %f %f %f %f %f %f" % (left_wheel_speed, right_wheel_speed,
                                        fs_speed, bs_speed, arm_leg_speed[0], arm_leg_speed[1], arm_leg_speed[2], arm_leg_speed[3])
                )
            else:
                self.arm_count[team][id] = 0
                self.relocate_arm_leg(team, id)
                custom_0 = '0 0 0 0 0 0 0 0'
                robot.getField('customData').setSFString(custom_0)

    def relocate_arm_leg(self, team, id):
        self.arm_count[team][id] = 0
        robot = self.robot[team][id]['node']

        Rotation = [1, 0, 0, 0]
        Rotation2 = [0, 0, 1,-1.57]
        laTranslation = [0, 0.07, 0]
        raTranslation = [0, 0.07, 0]
        laTranslation2 = [-0.238, 0.256, 0]
        raTranslation2 = [-0.238, 0.256, 0]
        llTranslation = [0, 0.01, 0]
        rlTranslation = [0, 0.01, 0]

        robot.getField('laTranslation').setSFVec3f(laTranslation)
        robot.getField('raTranslation').setSFVec3f(raTranslation)
        robot.getField('laRotation').setSFRotation(Rotation)
        robot.getField('raRotation').setSFRotation(Rotation)
        robot.getField('laTranslation2').setSFVec3f(laTranslation2)
        robot.getField('raTranslation2').setSFVec3f(raTranslation2)
        robot.getField('laRotation2').setSFRotation(Rotation2)
        robot.getField('raRotation2').setSFRotation(Rotation2)
        robot.getField('llTranslation').setSFVec3f(llTranslation)
        robot.getField('rlTranslation').setSFVec3f(rlTranslation)
        robot.getField('llRotation').setSFRotation(Rotation)
        robot.getField('rlRotation').setSFRotation(Rotation)
        robot.getField('llTranslation2').setSFVec3f(llTranslation)
        robot.getField('rlTranslation2').setSFVec3f(rlTranslation)
        robot.getField('llRotation2').setSFRotation(Rotation)
        robot.getField('rlRotation2').setSFRotation(Rotation)

    def relocate_all(self, team, id):
        self.relocate_arm_leg(team, id)
        self.arm_count[team][id] = 0
        self.arm_speed[team][id] = 0

    def relocate_all_every(self):
        for team in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                self.relocate_all(team, id)

    ## ball possession ##
    def get_ball_possession(self):
        for team in constants.TEAMS:
            for id in range(constants.NUMBER_OF_ROBOTS):
                if self.robot_in_field(team, id):
                    self.ball_possession[team][id] = self.check_ball_poss(team, id)
                    self.robot[team][id]['ball_possession'] = self.ball_possession[team][id]

    def check_ball_poss(self, team, id):
        ball_x = self.get_ball_position()[0]
        ball_y = self.get_ball_position()[1]
        ball_z = self.get_ball_position()[2]
        robot_pos = self.get_robot_posture(team, id)
        x = robot_pos[0]
        y = robot_pos[1]
        z = robot_pos[2]
        th = robot_pos[3]

        theta = th
        if (th > self.constants.PI):
            theta -= 2*self.constants.PI
        d_theta = abs(theta - math.atan2(ball_y-y, ball_x-x))
        if (d_theta > self.constants.PI):
            d_theta -= 2*self.constants.PI
        dist = math.sqrt((ball_y - y)*(ball_y - y)+(ball_x - x)*(ball_x - x))
        add = 0.1
        d_range = self.constants.ROBOT_SIZE[id]/2 + self.constants.BALL_RADIUS + add

        if ((dist < d_range) and (abs(d_theta) < self.constants.PI/4) and (abs(z - ball_z) < 0.01)):
            return True
        else:
            return False

    ## spotlight
    def ball_spotlight_stop(self):
        self.spotlight.setVelocity([0, 0, 0, 0, 0, 0])

    def ball_spotlight(self):
        p = self.ball.getPosition()
        v = self.ball.getVelocity()
        self.spotlight.getField('translation').setSFVec3f([p[0], 0.95 + (p[1]-0.05)*2/7, p[2]])
        self.spotlight.setVelocity([v[0], v[1]*2/7, v[2], 0, 0, 0])

    def random_friction(self):
        if self.time % (self.friction_interval*1000) == 0:
            self.current_friction = self.friction_fixed
            if self.friction_random:
                num = random.random()
                if num < 1/3:
                    self.current_friction = 0.1
                elif num < 2/3:
                    self.current_friction = 0.5
                else:
                    self.current_friction = 3.0
        floor_wheel = self.getFromDef('floor-wheel').getField('coulombFriction')
        floor_wheel.setMFFloat(0,self.current_friction)

    def run(self):
        config_file = open('../../config.json')
        config = json.loads(config_file.read())
        self.game_time = self.constants.DEFAULT_GAME_TIME_MS / self.constants.PERIOD_MS * self.constants.PERIOD_MS

        if config['rule']:
            if config['rule']['game_time']:
                self.game_time = config['rule']['game_time'] * 1000 / self.constants.PERIOD_MS * self.constants.PERIOD_MS
        else:
            print('"rule" section of \'config.json\' seems to be missing: using default options\n')
        print('Rules:\n')
        print('     game duration - ' + str(self.game_time / 1000) + ' seconds\n')

        # gets other options from 'config.json' (if no option is specified, default option is given)
        player_infos = []
        repeat = False
        synchronous_mode = True
        if config['tool']:
            if config['tool']['repeat']:
                repeat = config['tool']['repeat']

        if config['friction']:
            self.friction_random = config['friction']['randomize']
            self.friction_fixed = config['friction']['fixed_friction']
            self.friction_interval = config['friction']['interval']
        pn = self.getFromDef("DEF_AUDVIEW")
        pn.getField('follow').setSFString("")

        path_prefix = '../../'
        team_name = {}
        self.role_info = {}
        self.role_client = {}
        self.ready = [False] * 2  # TEAM_RED, TEAM_BLUE

        # gets the teams' information from 'config.json'
        for team in self.constants.TEAMS:
            if team == self.constants.TEAM_RED:
                tc = 'team_a'
                tc_op = 'team_b'
            else:
                tc = 'team_b'
                tc_op = 'team_a'
            # my team
            name = ''
            rating = 0  # rating is disabled
            exe = ''
            # keyboard = False
            if config[tc]:
                if config[tc]['name']:
                    name = config[tc]['name']
                if config[tc]['executable']:
                    exe = config[tc]['executable']
            # opponent
            name_op = ''
            rating_op = 0  # rating is disabled
            if config[tc_op]:
                if config[tc_op]['name']:
                    name_op = config[tc_op]['name']
            player_infos.append({
                'name': name,
                'rating': rating,
                'exe': path_prefix + exe,
                'role': team
            })

            if team == self.constants.TEAM_RED:
                print('Team A:\n')
            else:
                print('Team B:\n')
            print('  team name - ' + name + '\n')
            team_name[team] = name
            print('  executable - ' + exe + '\n')

            # create information for aiwc.get_info() in advance
            info = {}
            info['field'] = [self.constants.FIELD_LENGTH, self.constants.FIELD_WIDTH]
            info['goal'] = [self.constants.GOAL_DEPTH, self.constants.GOAL_WIDTH]
            info['penalty_area'] = [self.constants.PENALTY_AREA_DEPTH, self.constants.PENALTY_AREA_WIDTH]
            info['goal_area'] = [self.constants.GOAL_AREA_DEPTH, self.constants.GOAL_AREA_WIDTH]
            info['ball_radius'] = self.constants.BALL_RADIUS
            info['ball_mass'] = self.constants.BALL_MASS
            info['robot_size'] = self.constants.ROBOT_SIZE
            info['robot_height'] = self.constants.ROBOT_HEIGHT
            info['axle_length'] = self.constants.AXLE_LENGTH
            info['robot_body_mass'] = self.constants.ROBOT_BODY_MASS
            info['wheel_radius'] = self.constants.WHEEL_RADIUS
            info['wheel_mass'] = self.constants.WHEEL_MASS
            info['max_linear_velocity'] = self.constants.MAX_LINEAR_VELOCITY
            info['max_torque'] = self.constants.MAX_TORQUE
            info['number_of_robots'] = self.constants.NUMBER_OF_ROBOTS
            info['codewords'] = self.constants.CODEWORDS
            info['game_time'] = self.game_time / 1000
            info['team_info'] = [[['name_a', name], ['rating', rating]], [['name_b', name_op], ['rating', rating_op]]]
            info['key'] = random_string(self.constants.KEY_LENGTH)
            info['state_size'] = 5
            self.role_info[team] = info

        self.tcp_server = TcpServer(self.constants.SERVER_IP, self.constants.SERVER_PORT, self.constants.MAX_INSTANCES)
        self.ball = self.getFromDef(self.constants.DEF_BALL)
        self.time = 0
        self.random_friction()
        self.kickoff_time = self.time
        self.score = [0, 0]
        self.ball_ownership = self.constants.TEAM_RED  # red
        self.robot = [[0 for x in range(self.constants.NUMBER_OF_ROBOTS)] for y in range(2)]
        for t in self.constants.TEAMS:
            for id in range(self.constants.NUMBER_OF_ROBOTS):
                node = self.getFromDef(get_robot_name(self, t, id))
                self.robot[t][id] = {}
                self.robot[t][id]['node'] = node
                self.robot[t][id]['active'] = True
                self.robot[t][id]['touch'] = False
                self.robot[t][id]['ball_possession'] = False
        self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
        self.lock_all_robots(True)
        self.robot[self.constants.TEAM_RED][0]['active'] = True

        # start participants
        for player_info in player_infos:
            exe = player_info['exe']
            if not os.path.exists(exe):
                print('Participant controller not found: ' + exe)
            else:
                command_line = []
                if exe.endswith('.py'):
                    os.environ['PYTHONPATH'] += os.pathsep + os.path.join(os.getcwd(), 'player_py')
                    if sys.platform == 'win32':
                        command_line.append('python')
                command_line.append(exe)
                command_line.append(self.constants.SERVER_IP)
                command_line.append(str(self.tcp_server.get_port()))
                command_line.append(self.role_info[player_info['role']]['key'])
                print(command_line)
                subprocess.Popen(command_line)
        self.started = False
        self.success = []
        print('Waiting for player to be ready...')

        while True:
            
            sys.stdout.flush()
            self.tcp_server.spin(self)
            if not self.started:
                if all(self.ready):
                    print('Starting match.')
                    self.started = True
                    self.reset_ball(random.uniform(-self.constants.FIELD_LENGTH/2 + 2, self.constants.FIELD_LENGTH/2 - 2),
                                    random.uniform(self.constants.FIELD_WIDTH/2 - 1.5, -self.constants.FIELD_WIDTH/2 + 1.5))
                    self.ball_position = self.get_ball_position()
                    self.publish_current_frame(Game.GAME_START)
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                else:
                    if self.step(self.timeStep) == -1:
                        break
                    else:
                        self.waitReady += self.timeStep
                        if (self.waitReady == self.constants.WAIT_READY_MS):
                            print('Game could not be initiated. Need two players ready.')
                            return
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                continue

            if synchronous_mode:
                if all(self.sync):
                    self.sync[self.constants.TEAM_RED] = False
                    self.sync[self.constants.TEAM_BLUE] = False
                else:
                    continue

            self.random_friction()

            self.ball_position = self.get_ball_position()
            if self.time >= self.game_time:
                if repeat:
                    self.success.append(0)
                    if len(self.success) > 20:
                        self.success = self.success[1:]
                    self.setLabel(99, 'FAIL', 0.43, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.EPISODE_END)
                    self.reset_reason = Game.EPISODE_END
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    self.setLabel(99, '', 0.4, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.ball_ownership = self.constants.TEAM_RED
                    self.game_state = Game.STATE_KICKOFF
                    self.time = 0
                    self.kickoff_time = self.time
                    self.score = [0, 0]
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.reset_ball(random.uniform(-self.constants.FIELD_LENGTH/2 + 2, self.constants.FIELD_LENGTH/2 - 2),
                                    random.uniform(self.constants.FIELD_WIDTH/2 - 1.5, -self.constants.FIELD_WIDTH/2 + 1.5))
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                    self.publish_current_frame(Game.GAME_START)
                else:
                    self.setLabel(99, 'FAIL', 0.43, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.GAME_END)
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    break

            self.publish_current_frame()
            self.reset_reason = Game.NONE

            # update ball possession status of robots
            self.get_ball_possession()

            ball_x = self.ball_position[0]
            ball_y = self.ball_position[1]
            ball_z = self.ball_position[2]
            robot_pos = self.get_robot_posture(self.constants.TEAM_RED, 0)
            x = robot_pos[0]
            y = robot_pos[1]
            if self.robot[constants.TEAM_RED][0]['ball_possession']:
                if repeat:
                    self.success.append(1)
                    if len(self.success) > 20:
                        self.success = self.success[1:]
                    self.setLabel(99, 'SUCCESS', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.EPISODE_END)
                    self.reset_reason = Game.EPISODE_END
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    self.setLabel(99, '', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.ball_ownership = self.constants.TEAM_RED
                    self.game_state = Game.STATE_KICKOFF
                    self.time = 0
                    self.kickoff_time = self.time
                    self.score = [0, 0]
                    self.reset(self.constants.FORMATION_KICKOFF, self.constants.FORMATION_DEFAULT)
                    self.reset_ball(random.uniform(-self.constants.FIELD_LENGTH/2 + 2, self.constants.FIELD_LENGTH/2 - 2),
                                    random.uniform(self.constants.FIELD_WIDTH/2 - 1.5, -self.constants.FIELD_WIDTH/2 + 1.5))
                    self.lock_all_robots(True)
                    self.robot[self.constants.TEAM_RED][0]['active'] = True
                    if self.step(self.constants.WAIT_STABLE_MS) == -1:
                        break
                    self.publish_current_frame(Game.GAME_START)
                    if len(self.success) == 20 and sum(self.success)/20 > 0.95:
                        self.setLabel(99, 'FINISH', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                        self.publish_current_frame(Game.GAME_END)
                        self.stop_robots()
                        break
                else:
                    self.setLabel(99, 'SUCCESS', 0.35, 0.4, 0.3, 0xcc0000, 0, 'Impact')
                    self.publish_current_frame(Game.GAME_END)
                    self.stop_robots()
                    if self.step(self.constants.WAIT_END_MS) == -1:
                        break
                    break

            self.ball_spotlight()
            if not self.ball_in_field():
                self.ball_spotlight_stop()

            if self.step(self.timeStep, runTimer=True) == -1:
                break

controller = GameSupervisor()
controller.run()