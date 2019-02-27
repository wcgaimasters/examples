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
            self.game_time = info['game_time']
            self.number_of_robots = info['number_of_robots']

            self.field = info['field']
            self.goal = info['goal']
            self.penalty_area = info['penalty_area']
            self.goal_area = info['goal_area']
            self.resolution = info['resolution']

            self.ball_radius = info['ball_radius']
            self.ball_mass = info['ball_mass']

            self.robot_size = info['robot_size']
            self.robot_height = info['robot_height']
            self.axle_length = info['axle_length']
            self.robot_body_mass = info['robot_body_mass']

            self.wheel_radius = info['wheel_radius']
            self.wheel_mass = info['wheel_mass']

            self.max_linear_velocity = info['max_linear_velocity']
            self.max_torque = info['max_torque']
            self.codewords = info['codewords']

            # Print received constant variables to the console
            self.printConsole("======================================================")
            self.printConsole("Game Time: {} seconds".format(self.game_time))
            self.printConsole("# of robots: {} robots".format(self.number_of_robots))
            self.printConsole("======================================================")
            self.printConsole("Field Dimensions: {} m long, {} m wide".format(self.field[X], self.field[Y]))
            self.printConsole("Goal Dimensions: {} m deep, {} m wide".format(self.goal[X], self.goal[Y]))
            self.printConsole("Penalty Area Dimensions: {} m long, {} m wide".format(self.penalty_area[X], self.penalty_area[Y]))
            self.printConsole("Goal Area Dimensions: {} m long, {} m wide".format(self.goal_area[X], self.goal_area[Y]))
            self.printConsole("Image Resolution: {} x {}".format(self.resolution[X], self.resolution[Y]))
            self.printConsole("======================================================")
            self.printConsole("Ball Radius: {} m".format(self.ball_radius))
            self.printConsole("Ball Mass: {} kg".format(self.ball_mass))
            self.printConsole("======================================================")
            for i in range(self.number_of_robots):
                self.printConsole("Robot {}:".format(i))
                self.printConsole("  size: {} m x {} m".format(self.robot_size[i], self.robot_size[i]))
                self.printConsole("  height: {} m".format(self.robot_height[i]))
                self.printConsole("  axle length: {} m".format(self.axle_length[i]))
                self.printConsole("  body mass: {} kg".format(self.robot_body_mass[i]))
                self.printConsole("  wheel radius: {} m".format(self.wheel_radius[i]))
                self.printConsole("  wheel mass: {} kg".format(self.wheel_mass[i]))
                self.printConsole("  max linear velocity: {} m/s".format(self.max_linear_velocity[i]))
                self.printConsole("  max torque: {} N*m".format(self.max_torque[i]))
                self.printConsole("  codeword: {}".format(self.codewords[i]))
                self.printConsole("======================================================")
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
                self.printConsole("Game started : {}".format(f['time']))
            elif (f['reset_reason'] == SCORE_MYTEAM):
                self.printConsole("My team scored : {}".format(f['time']))
                self.printConsole("Current Score: {}".format(f['score']))
            elif (f['reset_reason'] == SCORE_OPPONENT):
                self.printConsole("Opponent scored : {}".format(f['time']))
                self.printConsole("Current Score: {}".format(f['score']))
            elif (f['reset_reason'] == HALFTIME):
                self.printConsole("Halftime")
            elif (f['reset_reason'] == EPISODE_END):
                self.printConsole("Episode ended.")
##############################################################################
            elif (f['reset_reason'] == GAME_END):
                self.printConsole("Game ended.")
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

        if 'half_passed' in f:
            self.printConsole("Halftime passed? {}".format(f['half_passed']))

        if 'game_state' in f:
            if (f['game_state'] == STATE_KICKOFF):
                self.printConsole("Kickoff [My kickoff? {}]".format(f['ball_ownership']))
            elif (f['game_state'] == STATE_GOALKICK):
                self.printConsole("Goalkick [My goalkick? {}]".format(f['ball_ownership']))
            elif (f['game_state'] == STATE_CORNERKICK):
                self.printConsole("Cornerkick [My cornerkick? {}]".format(f['ball_ownership']))
            elif (f['game_state'] == STATE_PENALTYKICK):
                self.printConsole("Penaltykick [My penaltykick? {}]".format(f['ball_ownership']))

        # Check the coordinates
        if 'coordinates' in f:
            myteam = f['coordinates'][MY_TEAM]
            opponent = f['coordinates'][OP_TEAM]
            ball =  f['coordinates'][BALL]

            self.printConsole("======================================================")
            self.printConsole("Ball: ({}, {})".format(ball[X], ball[Y]))
            self.printConsole("======================================================")

            # Try replacing 'myteam' with 'opponent' to check opponent robots' state
            for i in range(self.number_of_robots):
                self.printConsole("Robot {}:".format(i))
                self.printConsole("  position: ({}, {})".format(myteam[i][X], myteam[i][Y]))
                self.printConsole("  orientation: {}".format(myteam[i][TH]))
                self.printConsole("  activeness: {}".format(myteam[i][ACTIVE]))
                self.printConsole("  touch: {}".format(myteam[i][TOUCH]))
                self.printConsole("======================================================")

        if 'EOF' in f:
            if (f['EOF']):
##############################################################################
                #(virtual update())
                wheels = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
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
