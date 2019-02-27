#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random
import math
import os
import sys

import base64
import numpy as np

#from PIL import Image
from dqn_nn import NeuralNetwork

#reset_reason
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

#game_state
STATE_DEFAULT = 0
STATE_KICKOFF = 1
STATE_GOALKICK = 2
STATE_CORNERKICK = 3
STATE_PENALTYKICK = 4

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
ACTIVE = 3
TOUCH = 4

#path to your checkpoint
CHECKPOINT = os.path.join(os.path.dirname(__file__), 'dqn.ckpt')

class Received_Image(object):
    def __init__(self, resolution, colorChannels):
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels)) # rows, columns, colorchannels
    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0,len(received_parts)):
           dec_msg = base64.b64decode(self.received_parts[i].b64, '-_') # decode the base64 message
           np_msg = np.fromstring(dec_msg, dtype=np.uint8) # convert byte array to numpy array
           reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
           for j in range(0, self.received_parts[i].height): # y axis
               for k in range(0, self.received_parts[i].width): # x axis
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0] # blue channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1] # green channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2] # red channel

class SubImage(object):
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64

class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.subimages = None
        self.coordinates = None
        self.half_passed = None

class Component(ApplicationSession):
    """
    AI Base + Deep Q Network example
    """

    def __init__(self, config):
        ApplicationSession.__init__(self, config)

    def printConsole(self, message):
        print(message)
        sys.__stdout__.flush()

    def onConnect(self):
        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, number_of_robots
            #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
            #       ball_radius, ball_mass,
            #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
            #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
            #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
            # self.game_time = info['game_time']
            # self.number_of_robots = info['number_of_robots']

            # self.field = info['field']
            # self.goal = info['goal']
            # self.penalty_area = info['penalty_area']
            # self.goal_area = info['goal_area']
            self.resolution = info['resolution']

            # self.ball_radius = info['ball_radius']
            # self.ball_mass = info['ball_mass']

            # self.robot_size = info['robot_size']
            # self.robot_height = info['robot_height']
            # self.axle_length = info['axle_length']
            # self.robot_body_mass = info['robot_body_mass']

            # self.wheel_radius = info['wheel_radius']
            # self.wheel_mass = info['wheel_mass']

            self.max_linear_velocity = info['max_linear_velocity']
            # self.max_torque = info['max_torque']
            # self.codewords = info['codewords']

            self.colorChannels = 3 # nf in dqn_main.py
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            self._frame = 0
            self.Q = NeuralNetwork(None, CHECKPOINT, False) # 2nd term: False to start training from scratch, use CHECKPOINT to load a checkpoint
            self.wheels = [0 for _ in range(10)]
            return
##############################################################################

        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            try:
                self.sub = yield self.subscribe(self.on_event, args.key)
            except Exception as e2:
                self.printConsole("Error: {}".format(e2))

        init_variables(self, info)

        try:
            yield self.call(u'aiwc.ready', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            self.printConsole("I am ready for the game!")

    @inlineCallbacks
    def on_event(self, f):

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return

        def set_action(robot_id, action_number):
            if action_number == 0:
                self.wheels[2*robot_id] = 0.75
                self.wheels[2*robot_id + 1] = 0.75
                # Go Forward with fixed velocity
            elif action_number == 1:
                self.wheels[2*robot_id] = 0.75
                self.wheels[2*robot_id + 1] = 0.5
                # Turn
            elif action_number == 2:
                self.wheels[2*robot_id] = 0.75
                self.wheels[2*robot_id + 1] = 0.25
                # Turn
            elif action_number == 3:
                self.wheels[2*robot_id] = 0.75
                self.wheels[2*robot_id + 1] = 0
                # Turn
            elif action_number == 4:
                self.wheels[2*robot_id] = 0.5
                self.wheels[2*robot_id + 1] = 75
                # Turn
            elif action_number == 5:
                self.wheels[2*robot_id] = 0.25
                self.wheels[2*robot_id + 1] = 0.75
                # Turn
            elif action_number == 6:
                self.wheels[2*robot_id] = 0
                self.wheels[2*robot_id + 1] = 0.75
                # Turn
            elif action_number == 7:
                self.wheels[2*robot_id] = -0.75
                self.wheels[2*robot_id + 1] = -0.75
                # Go Backward with fixed velocity
            elif action_number == 8:
                self.wheels[2*robot_id] = -0.1
                self.wheels[2*robot_id + 1] = 0.1
                # Spin
            elif action_number == 9:
                self.wheels[2*robot_id] = 0.1
                self.wheels[2*robot_id + 1] = -0.1
                # Spin
            elif action_number == 10:
                self.wheels[2*robot_id] = 0
                self.wheels[2*robot_id + 1] = 0
                # Do not move

        # initiate empty frame
        received_frame = Frame()
        received_subimages = []

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'half_passed' in f:
            received_frame.half_passed = f['half_passed']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            # Comment the next lines if you don't need to use the image information
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))
            self.image.update_image(received_subimages)
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']
        if 'EOF' in f:
            self.end_of_frame = f['EOF']

        #self.printConsole(received_frame.time)
        #self.printConsole(received_frame.score)
        #self.printConsole(received_frame.reset_reason)
        #self.printConsole(self.end_of_frame)

        if (self.end_of_frame):
            self._frame += 1

            # To get the image at the end of each frame use the variable:
            #self.printConsole(self.image.ImageBuffer)

##############################################################################
            #(virtual update() in random_walk.cpp)

            # State

            # If you want to use the image as the input for your network
            # You can use pillow: PIL.Image to get and resize the input frame as follows
            #img = Image.fromarray((self.image.ImageBuffer/255).astype('uint8'), 'RGB') # Get normalized image as a PIL.Image object
            #resized_img = img.resize((NEW_X,NEW_Y))
            #final_img = np.array(resized_img)

            # Example: using the normalized coordinates for robot 0 and ball
            position = [round(received_frame.coordinates[MY_TEAM][0][X]/2.05, 2), round(received_frame.coordinates[MY_TEAM][0][Y]/1.35, 2),
                        round(received_frame.coordinates[MY_TEAM][0][TH]/(2*math.pi), 2), round(received_frame.coordinates[BALL][X]/2.05, 2),
                        round(received_frame.coordinates[BALL][Y]/1.35, 2)]

            # Action
            action = self.Q.BestAction(np.array(position)) # using CNNs use final_img as input

            # Set robot wheels
            set_action(0, action)
            set_wheel(self, self.wheels)

##############################################################################

            if(received_frame.reset_reason == GAME_END):

##############################################################################
                #(virtual finish() in random_walk.cpp)
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave
                yield self.sub.unsubscribe()
                try:
                    yield self.leave()
                except Exception as e:
                    self.printConsole("Error: {}".format(e))
##############################################################################

            self.end_of_frame = False


    def onDisconnect(self):
        if reactor.running:
            reactor.stop()

if __name__ == '__main__':

    try:
        unicode
    except NameError:
        # Define 'unicode' for Python 3
        def unicode(s, *_):
            return s

    def to_unicode(s):
        return unicode(s, "utf-8")

    parser = argparse.ArgumentParser()
    parser.add_argument("server_ip", type=to_unicode)
    parser.add_argument("port", type=to_unicode)
    parser.add_argument("realm", type=to_unicode)
    parser.add_argument("key", type=to_unicode)
    parser.add_argument("datapath", type=to_unicode)

    args = parser.parse_args()

    ai_sv = "rs://" + args.server_ip + ":" + args.port
    ai_realm = args.realm

    # create a Wamp session object
    session = Component(ComponentConfig(ai_realm, {}))

    # initialize the msgpack serializer
    serializer = MsgPackSerializer()

    # use Wamp-over-rawsocket
    runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])

    runner.run(session, auto_reconnect=False)
