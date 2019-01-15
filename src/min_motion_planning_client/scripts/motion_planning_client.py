#! /usr/bin/env python
# THIS IS FOR THE SEPARATE SETUP REPO
from __future__ import print_function  # Lets you print like Python 3
import rospy
import actionlib
# import simple_action_example.msg
# from motion_action_server import MoveRobot.msg
# import MoveRobot.msg
# import motion_action_server.msg
# import motion_msgs.msg  # import MoveRobotAction
import os

from PIL import Image

import motion_msgs
from motion_msgs.msg import MoveRobotAction
from force_msgs.msg import LoadCellForces32


class Robot:

    # .015 blue
    # .02 green
    # .02 red
    zZ = .015
    zR = .015  # was .02
    zG = .015
    zB = .015
    lastX = 0
    lastY = -.8
    lastZ = .4
    lastRoll = 0
    lastPitch = 180
    lastYaw = 0
    lastTool = 0
    zClearance = .04
    zTouching = .015


    def __init__(self):
        self.client = actionlib.SimpleActionClient('motion',
                                                   motion_msgs.msg.MoveRobotAction)

        self.force_sub = rospy.Subscriber('/force_data', LoadCellForces32, self.forceCB)

    def forceCB(self, data):  # function to prcess force data
        print("forcecb working")
        zeroThresh = .5  # .5 is half way point for force
        abortThresh = .5
        # print (data)

        if data.cellA > zeroThresh or data.cellB > zeroThresh or data.cellC > zeroThresh:
            print(data.cellA)

        if data.cellA > abortThresh or data.cellB > abortThresh or data.cellC > abortThresh:
            ROS_WARN("Too much force!")
            self.client.cancel_all_goals()

    def clear(self):
        # Waits until the action server has started up and started
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = motion_msgs.msg.MoveRobotGoal(self.lastX, self.lastY, self.zClearance, self.lastRoll, self.lastPitch,
                                             self.lastYaw, self.lastTool, 2)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

    # move z only to an externally measured height (zTouching)
    def touch(self):
        # Waits until the action server has started up and started
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = motion_msgs.msg.MoveRobotGoal(self.lastX, self.lastY, self.zTouching, self.lastRoll, self.lastPitch,
                                             self.lastYaw, self.lastTool, 2)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

    def changeTool(self, tool):

        self.lastTool = tool

        # Waits until the action server has started up and started
        # listening for goals. (So the goals aren't ignored.)
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        # Z sad
        goal = motion_msgs.msg.MoveRobotGoal(self.lastX, self.lastY, .4, self.lastRoll, self.lastPitch,
                                             self.lastYaw, tool, 2)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

    def moveXYZ(self, x, y, z):
        self.lastX = x
        self.lastY = y
        self.lastZ = z

        self.client.wait_for_server()
        goal = motion_msgs.msg.MoveRobotGoal(x, y, z, self.lastRoll, self.lastPitch, self.lastYaw, self.lastTool, 2)
        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

    # moves arm to a new x,y position without changing z
    def moveXY(self, x, y):
        self.lastX = x
        self.lastY = y

        self.client.wait_for_server()
        goal = motion_msgs.msg.MoveRobotGoal(x, y, self.lastZ, self.lastRoll, self.lastPitch, self.lastYaw,
                                             self.lastTool, 2)

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

    # moves arm to a specified position
    def move(self, x, y, z, roll, pitch, yaw, tool):

        self.lastX = x
        self.lastY = y
        self.lastZ = z
        self.lastRoll = roll
        self.lastPitch = pitch
        self.lastYaw = yaw
        self.lastTool = tool

        # Waits until the action server has started up and started
        # listening for goals. (So the goals aren't ignored.)
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = motion_msgs.msg.MoveRobotGoal(x, y, z, roll, pitch, yaw, tool, 2)
        # goal = motion_msgs.msg.MoveRobotGoal()
        # goal.z=.4

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # client.cancel_all_goals()
        print("move")
        # print(self.client.get_feedback())
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()


def main_node():
    myRobot = Robot()

    debug = True

    if debug == False:

        print("debug is false")

        # Sets a relative 'zero' for motion planning
        xOrigin = -.78  # +.15  # rospy.get_param('workcell/canvas_x') -.75
        yOrigin = -.72  # +.15  # rospy.get_param('workcell/canvas_y') -.72
        boardz = rospy.get_param('workcell/canvas_z')

        scalingFactor = .004  # 1=Meter .01=cm

        zTouching = .015
        currentTool = "tool_red"

        scriptDir = os.path.dirname(__file__)
        fileName = "BMOtrace2.png"
        # fileName="3X3Cal.png"
        impath = os.path.join(scriptDir, '../../../../seurobot_ws/image_script/')
        print("reading image")
        img = Image.open(impath+fileName)
        img = img.convert('RGB')
        width, height = img.size
        print("Image done reading")
        # painter.move(boardx, boardy, zClearance, 0 , 180, 0, currentTool)
        print("Moving to origin")

        done = myRobot.move(xOrigin, yOrigin, .2, 0, 180, 0, "tool_red")

        print("waiting..")

        if done:
            print("STARTING RUN")

        else:
            print("mode ERROR")
    else:
        try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            # x=-.5 y=-.6
            #              x,  y,  z,  r,  p,  y,  tool
            # z rotation off by 180 -28?

            # z .025 red
            # z .0225 blue

            # x-.4 y-1.2
            # .015 blue
            # .02 green
            # .02 red

            # x-.5 y-1

            # GOT TO 65 57

            result = myRobot.move(-.78, -.72, .4015, 0, 180, 0, "tool_red")

            print(result)

        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)

            # Prints out the result of executing the action
    rospy.spin()


if __name__ == '__main__':
    print("Client Running...")
    boardx = rospy.get_param('workcell/canvas_x')
    boardy = rospy.get_param('workcell/canvas_y')
    boardz = rospy.get_param('workcell/canvas_z')

    print("Board Top Left: x=", boardx, "y=", boardy, "z=", boardz)

    # rospy.init_node('force_sub', anonymous=True)
    # sub=rospy.Subscriber('force_data', LoadCellForces32, forceCB)

    rospy.init_node('main_node_py', anonymous=True)
    main_node()
