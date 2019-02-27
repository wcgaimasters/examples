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
import sys

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

class Component(ApplicationSession):
    """
    AI Base + Skeleton
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
            self.number_of_robots = info['number_of_robots']

            # self.field = info['field']
            # self.goal = info['goal']
            # self.penalty_area = info['penalty_area']
            # self.goal_area = info['goal_area']
            # self.resolution = info['resolution']

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

        if 'reset_reason' in f:
            if (f['reset_reason'] == GAME_START):
                self.printConsole("Game started : " + str(f['time']))
            if (f['reset_reason'] == SCORE_MYTEAM):
                self.printConsole("My team scored : " + str(f['time']))
            elif (f['reset_reason'] == SCORE_OPPONENT):
                self.printConsole("Opponent scored : " + str(f['time']))
            if (f['reset_reason'] == EPISODE_END):
                self.printConsole("Episode ended.")
            if (f['reset_reason'] == GAME_END):
                self.printConsole("Game ended.")

##############################################################################
                #(virtual finish())
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

        # if 'coordinates' in f:
            # myteam = f['coordinates'][MY_TEAM]
            # opponent = f['coordinates'][OP_TEAM]
            # ball =  f['coordinates'][BALL]

            # myteam0_x = f['coordinates'][MY_TEAM][0][X]
            # myteam0_y = f['coordinates'][MY_TEAM][0][Y]
            # myteam0_th = f['coordinates'][MY_TEAM][0][TH]
            # myteam0_act = f['coordinates'][MY_TEAM][0][ACTIVE]
            # myteam0_tou = f['coordinates'][MY_TEAM][0][TOUCH]
            # if (myteam0_tou == True):
            #   self.printConsole("My robot 0 touched the ball!")

        if 'EOF' in f:
            if (f['EOF']):

##############################################################################
                #(virtual update())
                wheels = []
                for i in range(self.number_of_robots):
                    wheels.append(self.max_linear_velocity[i])
                    wheels.append(self.max_linear_velocity[i])
                set_wheel(self, wheels)
##############################################################################

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
