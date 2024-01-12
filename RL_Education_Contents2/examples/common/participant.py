import json
import socket
import sys
import math

class Frame():
    def __init__(self):
        self.time = None
        self.coordinates = None
        self.state = None
        self.state_size = None
        self.reward_continuous = None
        self.reward_binary = None
        self.reward_sparse = None

    # coordinates
    MY_TEAM = 0
    OP_TEAM = 1
    BALL = 2
    X = 0
    Y = 1
    Z = 2
    TH = 3
    ACTIVE = 4
    TOUCH = 5
    BALL_POSSESSION = 6

class Game():
    # reset_reason
    NONE = 0
    GAME_START = 1
    SCORE_MYTEAM = 2
    SCORE_OPPONENT = 3
    GAME_END = 4
    DEADLOCK = 5
    GOALKICK = 6
    CORNERKICK = 7
    PENALTYKICK = 8
    HALFTIME = 9
    EPISODE_END = 10

    # game_state
    STATE_DEFAULT = 0
    STATE_KICKOFF = 1
    STATE_GOALKICK = 2
    STATE_CORNERKICK = 3
    STATE_PENALTYKICK = 4

class Participant():
    def __init__(self):
        self.host = sys.argv[1]
        self.port = int(sys.argv[2])
        self.key = sys.argv[3]
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def send(self, message, arguments=[]):
        message = 'aiwc.' + message + '("%s"' % self.key
        for argument in arguments:
            if isinstance(argument, str):  # string
                message += ', "%s"' % argument
            else:  # number
                message += ', %s' % argument
        message += ')'
        try:
            self.socket.sendall(message.encode())
        except socket.error:
            self.socket.close()
            exit(0)

    def receive(self):
        try:
            data = self.socket.recv(4096)
        except socket.error:
            self.socket.close()
            exit(0)
        if not data:  # the connection was likely closed because the simulation terminated
            exit(0)
        return data.decode()

    def create_frame_object(self, f):
        # initiate empty frame
        frame = Frame()
        if 'time' in f:
            frame.time = f['time']
        if 'coordinates' in f:
            frame.coordinates = f['coordinates']
        if 'state_absolute' in f:
            frame.state_absolute = f['state_absolute']
        if 'state_relative' in f:
            frame.state_relative = f['state_relative']
        if 'state_mix' in f:
            frame.state_mix = f['state_mix']

        if 'state_absolute_wf' in f:
            frame.state_absolute_wf = f['state_absolute_wf']
        if 'state_relative_wf' in f:
            frame.state_relative_wf = f['state_relative_wf']
        if 'state_mix_wf' in f:
            frame.state_mix_wf = f['state_mix_wf']
            
        if 'state_size' in f:
            frame.state_size = f['state_size']
        if 'reward_continuous' in f:
            frame.reward_continuous = f['reward_continuous']
        if 'reward_binary' in f:
            frame.reward_binary = f['reward_binary']
        if 'reward_sparse' in f:
            frame.reward_sparse = f['reward_sparse']

        return frame

    def set_speeds(self, speeds):
        self.send('set_speeds', speeds)

    def printConsole(self, message):
        print(message)
        sys.__stdout__.flush()

    def check_frame(self, frame):  # you should override this method
        # if frame.reset_reason == Game.GAME_END:
        #     return False
        return True

    def init(self, info):  # you should override this method
        print("init() method called")

    def update(self, frame):  # you should override this method
        print("update() method called")

    def finish(self, frame):  # you should override this method
        print("finish() method called")

    def run(self):
        self.send('get_info')
        info = self.receive()

        self.init(json.loads(info))

        self.send('ready')
        data = ''
        while True:
            data += self.receive()
            if data and data[-1] == '}':  # make sure we received complete frame
                # data could contain multiple concatenated frames
                try:
                    frames = json.loads("[{}]".format(data.replace('}{', '},{')))
                    finished = False
                    for frame in frames:
                        frameObject = self.create_frame_object(frame)
                        if frame and self.check_frame(frameObject):  # return False if we need to quit
                            self.update(frameObject)
                        else:
                            self.finish(frameObject)
                            finished = True
                            break

                    if finished:
                        break
                except ValueError:
                    sys.stderr.write("Error: participant.py: Invalid JSON object.\n")
                data = ''
