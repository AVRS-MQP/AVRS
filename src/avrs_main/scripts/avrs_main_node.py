#! /usr/bin/env python

from __future__ import print_function # Lets you print like Python 3
import rospy
import actionlib
#import simple_action_example.msg
#from motion_action_server import MoveRobot.msg
#import MoveRobot.msg
#import motion_action_server.msg
import motion_action_server_msgs
from motion_action_server_msgs.msg import MoveRobotAction

def avrs_main_node():
    # SimpleActionClient construction, targeting the fibonacci topic of type Fibonacci
    client = actionlib.SimpleActionClient('motion',
                                          motion_action_server_msgs.msg.MoveRobotAction)

    # Waits until the action server has started up and started
    # listening for goals. (So the goals aren't ignored.)
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = motion_action_server_msgs.msg.MoveRobotAction()
    goal.x=1


    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    print("Client Running")
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('avrs_main_node_py')
        result = avrs_main_node()

        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
