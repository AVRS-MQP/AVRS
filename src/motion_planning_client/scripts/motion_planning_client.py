#! /usr/bin/env python

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

    # values for RGB
    offsetXR = 0
    offsetYR = 0
    offsetXG = .001
    offsetYG = .002
    offsetXB = -.0005
    offsetYB = .003

    # values for Black Cyan Yellow
    # offsetXR = 0
    # offsetYR = 0
    # offsetXG = 0.000385+.00005
    # offsetYG = 0.0015
    # offsetXB = -0.00215
    # offsetYB = 0.0035


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

        # if(tool=="tool_red"):
            # tool="tool_change_red"
        # if(tool=="tool_green"):
            # tool="tool_change_green"
        # if(tool=="tool_blue"):
            # tool="tool_change_blue"

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

    def moveXY(self, x, y):
        self.lastX = x
        self.lastY = y

        self.client.wait_for_server()
        goal = motion_msgs.msg.MoveRobotGoal(x, y, self.lastZ, self.lastRoll, self.lastPitch, self.lastYaw, self.lastTool, 2)
        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()

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

    def drawPoint(self, x, y):
        self.lastX = x
        self.lastY = y

        # Waits until the action server has started up and started
        # listening for goals. (So the goals aren't ignored.)
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = motion_msgs.msg.MoveRobotGoal(x, y, self.lastZ, self.lastRoll, self.lastPitch, self.lastYaw, self.lastTool, 3)
        # goal = motion_msgs.msg.MoveRobotGoal()
        # goal.z=.4

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # client.cancel_all_goals()
        print("feedback")
        # print(self.client.get_feedback())
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
        return self.client.get_result()


def printImage():
    print("in print image")


def main_node():
    myRobot = Robot()

    debug = False

    if (debug == False):

        print("debug is false")
        mode="eachcolorfast"

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

        colors = [
            "WHITE",
            # "BLACK",

            "BLUE",
            "RED",
            "GREEN",
            # "CYAN",
            # "MAGENTA",
            # "YELLOW",
            # "BROWN"
        ]
        print("waiting..")

        if done:
            print("STARTING RUN")
            if mode == "inorder":
                for x in range(width):
                    for y in range(height):
                        r, g, b = img.getpixel((x, y))
                        print("\nPrinting:")
                        print("X%= {}%".format(int(100.0 * x / width)))
                        print("Y%= {}%".format(int(100.0 * y / height)))
                        print("On x=", x, "y=", y, "Color=", r, g, b)

                        currentX = xOrigin + (x * scalingFactor * 1)
                        currentY = yOrigin + (y * scalingFactor * -1)

                        done = myRobot.clear()
                        if done:
                            done = myRobot.moveXY(currentX, currentY)
                            if done:
                                if r == 255 and g == 255 and b == 255 and "WHITE" in colors:  # WHITE
                                    done = 1
                                elif r == 255 and g == 0 and b == 0 and "RED" in colors:  # RED
                                    myRobot.zTouching=myRobot.zR
                                    if myRobot.changeTool("tool_red"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 0 and g == 255 and b == 0 and "GREEN" in colors:  # GREEN
                                    myRobot.zTouching=myRobot.zG
                                    if myRobot.changeTool("tool_green"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 0 and g == 0 and b == 255 and "BLUE" in colors:  # BLUE
                                    myRobot.zTouching=myRobot.zB
                                    if myRobot.changeTool("tool_blue"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 0 and g == 255 and b == 255 and "CYAN" in colors:  # CYAN
                                    if myRobot.changeTool("tool_red"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 255 and g == 0 and b == 255 and "MAGENTA" in colors:  # MAGENTA
                                    if myRobot.changeTool("tool_green"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 255 and g == 255 and b == 0 and "YELLOW" in colors:  # YELLOW
                                    if myRobot.changeTool("tool_blue"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 0 and g == 0 and b == 0 and "BLACK" in colors:  # BLACK
                                    if myRobot.changeTool("tool_red"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                elif r == 165 and g == 42 and b == 42 and "BROWN" in colors:  # BROWN
                                    if myRobot.changeTool("tool_green"):
                                        if myRobot.touch():
                                            myRobot.clear()
                                        else:
                                            print("ERROR: Couldn't touch")
                                    else:
                                        print("ERROR: Couldn't change tool")
                                else:  # SKIP
                                    done = 1
                            else:
                                print("ERROR: Couldn't move xy")
                        else:
                            print("ERROR: Couldn't clear")
            elif mode == "eachcolor":
                for i in range(len(colors)):
                    print("Changing tool")
                    if colors[i] == "RED" or colors[i] == "NONE" or colors[i] == "BLACK":
                        myRobot.changeTool("tool_red")
                    elif colors[i] == "GREEN" or colors[i] == "CYAN" or colors[i] == "BROWN":
                        myRobot.changeTool("tool_green")
                    elif colors[i] == "BLUE" or colors[i] == "YELLOW":
                        myRobot.changeTool("tool_blue")
                    print("STARTING RUN")

                    for x in range(width):
                        for y in range(height):
                            r, g, b = img.getpixel((x, y))
                            print("\nPrinting:")
                            print("X%= {}%".format(int(100.0 * x / width)))
                            print("Y%= {}%".format(int(100.0 * y / height)))
                            print("On x=", x, "y=", y, "Color=", r, g, b)

                            currentX = xOrigin + (x * scalingFactor * 1)
                            currentY = yOrigin + (y * scalingFactor * -1)

                            if done:
                                if r == 255 and g == 0 and b == 0 and colors[i] == "RED":  # RED
                                    myRobot.zTouching=myRobot.zR
                                    if myRobot.lastTool == "tool_red":
                                        if myRobot.moveXY(currentX+myRobot.offsetXR, currentY+myRobot.offsetYR):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 0 and g == 255 and b == 0 and colors[i] == "GREEN":  # GREEN
                                    myRobot.zTouching=myRobot.zG
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.moveXY(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 0 and g == 0 and b == 255 and colors[i] == "BLUE":  # BLUE
                                    myRobot.zTouching=myRobot.zB
                                    if myRobot.lastTool == "tool_blue":
                                        if myRobot.moveXY(currentX+myRobot.offsetXB, currentY+myRobot.offsetYB):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 0 and g == 255 and b == 255 and colors[i] == "CYAN":  # CYAN
                                    if myRobot.lastTool == "tool_red":
                                        if myRobot.moveXY(currentX+myRobot.offsetXR, currentY+myRobot.offsetYR):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 255 and g == 0 and b == 255 and colors[i] == "MAGENTA":  # MAGENTA
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.moveXY(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 255 and g == 255 and b == 0 and colors[i] == "YELLOW":  # YELLOW
                                    if myRobot.lastTool == "tool_blue":
                                        if myRobot.moveXY(currentX+myRobot.offsetXB, currentY+myRobot.offsetYB):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 0 and g == 0 and b == 0 and colors[i] == "BLACK":  # BLACK
                                    if myRobot.lastTool == "tool_red":
                                        if myRobot.moveXY(currentX+myRobot.offsetXR, currentY+myRobot.offsetYR):
                                            if myRobot.touch():
                                                myRobot.clear()
                                elif r == 165 and g == 42 and b == 42 and colors[i] == "BROWN":  # BROWN
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.moveXY(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            if myRobot.touch():
                                                myRobot.clear()
                                else:  # SKIP
                                    done = 1

            elif mode == "eachcolorfast":
                for i in range(len(colors)):
                    print("Changing tool")
                    if colors[i] == "RED" or colors[i] == "X" or colors[i] == "BLACK":
                        myRobot.changeTool("tool_red")
                    elif colors[i] == "GREEN" or colors[i] == "CYAN" or colors[i] == "BROWN":
                        myRobot.changeTool("tool_green")
                    elif colors[i] == "BLUE" or colors[i] == "YELLOW":
                        myRobot.changeTool("tool_blue")
                    print("STARTING RUN")


                    # x22 y 4

                    for x in range(width):
                        for y in range(height):
                            r, g, b = img.getpixel((x, y))
                            print("\nPrinting:")
                            print("X%= {}%".format(int(100.0 * x / width)))
                            print("Y%= {}%".format(int(100.0 * y / height)))
                            print("On x=", x, "y=", y, "Color=", r, g, b)

                            currentX = xOrigin + (x * scalingFactor * 1)
                            currentY = yOrigin + (y * scalingFactor * -1)

                            if done:
                                if r == 255 and g == 0 and b == 0 and colors[i] == "RED":  # RED
                                    myRobot.zTouching = myRobot.zR
                                    if myRobot.lastTool == "tool_red":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXR, currentY+myRobot.offsetYR):
                                            print("point")
                                elif r == 0 and g == 255 and b == 0 and colors[i] == "GREEN":  # GREEN
                                    myRobot.zTouching = myRobot.zG
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            print("point")
                                elif r == 0 and g == 0 and b == 255 and colors[i] == "BLUE":  # BLUE
                                    myRobot.zTouching = myRobot.zB
                                    if myRobot.lastTool == "tool_blue":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXB, currentY+myRobot.offsetYB):
                                            print("point")
                                elif r == 0 and g == 255 and b == 255 and colors[i] == "CYAN":  # CYAN
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            print("point")
                                elif r == 255 and g == 0 and b == 255 and colors[i] == "MAGENTA":  # MAGENTA
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            print("point")
                                elif r == 255 and g == 255 and b == 0 and colors[i] == "YELLOW":  # YELLOW
                                    if myRobot.lastTool == "tool_blue":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXB, currentY+myRobot.offsetYB):
                                            print("point")
                                elif r == 0 and g == 0 and b == 0 and colors[i] == "BLACK":  # BLACK
                                    if myRobot.lastTool == "tool_red":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXR, currentY+myRobot.offsetYR):
                                            print("point")
                                elif r == 165 and g == 42 and b == 42 and colors[i] == "BROWN":  # BROWN
                                    if myRobot.lastTool == "tool_green":
                                        if myRobot.drawPoint(currentX+myRobot.offsetXG, currentY+myRobot.offsetYG):
                                            print("point")
                                else:  # SKIP
                                    done = 1
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
