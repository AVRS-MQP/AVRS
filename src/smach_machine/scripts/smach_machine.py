#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

# from motion_functions import *
import actionlib_tutorials_msgs

import motion_msgs
from motion_msgs.msg import *
from force_msgs.msg import LoadCellForces32

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Quaternion
import numpy as np

# from . import Robot

import math
import tf
import geometry_msgs.msg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from geometry_msgs.msg import PoseStamped


def get_pose_from_tf(base_frame, pose_frame):
    listener = tf.TransformListener()
    # print("A")
    while not rospy.is_shutdown():
        # print("B")
        try:
            # print("C")
            result = listener.lookupTransform(base_frame, pose_frame, rospy.Time(0))
            print("RESULT")
            print(result)

            return result
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # print("ERROR")
            continue


# define state SetCalib
# based on current tool, calibrates force sensors and moves into the camera pose
class SetCalib(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'],
                             input_keys=['tool_in'],
                             output_keys=['tool_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SetCalib')
        if userdata.tool_in < 3:
            userdata.tool_out = userdata.tool_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state FindCar
class CameraPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['tool_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CameraPose')
        rospy.loginfo('Counter = %f' % userdata.tool_in)
        return 'outcome1'


# define state FindCar
class FindCar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['tool_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FindCar')
        rospy.loginfo('Exiting state machine')
        rospy.loginfo(userdata.tool_in)
        return 'outcome1'


# define state MoveArm
class MoveArm(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['move_pose'])

    def execute(self, userdata):
        pub = rospy.Publisher('smach_target', Pose, queue_size=1)
        print("EXECUTING")
        rospy.loginfo(userdata.move_pose)
        # pub.publish(userdata.move_pose)  # TODO

        client = actionlib.SimpleActionClient('motion',
                                              motion_msgs.msg.MoveRobotQuatAction)

        print("Waiting for server")
        client.wait_for_server()

        # Creates a goal to send to the action server.

        # localPose = geometry_msgs.Pose()

        tool = "vac"
        localPose = userdata.move_pose

        output = PoseStamped()

        output.pose.position = userdata.move_pose.pose.position
        output.pose.orientation = userdata.move_pose.pose.orientation
        print("HERE_A")

        # TEST
        (trans, rot) = get_pose_from_tf("base_link", "flap_raw")

        print("HERE_B")

        # THIS IS HOW TO PULL XYZRPY FROM POSE
        print("Pose")
        print(userdata.move_pose)
        # x = userdata.move_pose.pose.position.x
        # y = userdata.move_pose.pose.position.y
        # z = userdata.move_pose.pose.position.z
        # roll = userdata.move_pose.pose.orientation[0]
        # pitch = userdata.move_pose.pose.orientation[1]
        # yaw = userdata.move_pose.pose.orientation[2]
        # w = userdata.move_pose.pose.orientation[3]

        # THIS IS HOW TO PULL XYZRPY FROM TF
        x = trans[0]
        y = trans[1]
        z = trans[2]

        roll = rot[0]
        pitch = rot[1]
        yaw = rot[2]
        w = rot[3]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print("target")

        # force alignment
        # roll = 1.5708
        # pitch = 0
        # yaw=1.5708

        # shouldn't be needed if userdata.pose is correct?
        # q_rot = quaternion_from_euler(userdata.move_pose.pose.orientation[1], userdata.move_pose.pose.orientation[2], userdata.move_pose.pose.orientation[3])

        # print(x,y,z,roll,pitch,yaw)

        print("OUTPUT", output)

        # a=np.array([output.pose.position.x,output.pose.position.y, output.pose.position.z,output.pose.orientation.x,output.pose.orientation.y,output.pose.orientation.z,output.pose.orientation.w])
        # TODO FIGURE OUT WHY A POSE CAN"T GET PASSED INTO CUSTOM POSE ACTION MSG
        # goal = motion_msgs.msg.MoveRobotPoseGoal(output, tool, 2)
        goal = motion_msgs.msg.MoveRobotQuatGoal(x, y, z, roll, pitch, yaw, w, tool, 2)

        # goal = motion_msgs.msg.MoveRobotGoal()
        # goal.z=.4

        # Sends the goal to the action server.
        client.send_goal(goal)

        return 'outcome1'


# define state GetCarInfo
class GetCarInfo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'],
                             input_keys=['con_in'],
                             output_keys=['con_out'])
        # temp variable, recursion forever using just userdata
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state GetCarInfo')
        if self.counter < 2:
            userdata.con_out = userdata.con_in + 1
            self.counter = self.counter + 1
            return 'outcome1'
        else:
            return 'outcome2'


# # define state LocateFlap
class LocateFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['con_out'],
                             output_keys=['con_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state LocateFlap')
        rospy.loginfo('Current Counter Value: %f' % userdata.con_out)
        userdata.con_in = userdata.con_out
        return 'outcome1'


def main():
    rospy.init_node('smach_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome6'])
    sm.userdata.current_tool = 0
    sm.userdata.con_in = 0
    sm.userdata.con_out = 0
    sm.userdata.robot_pose = PoseStamped()

    sm.userdata.robot_pose.pose.position.x = 1.5
    sm.userdata.robot_pose.pose.position.y = .5
    sm.userdata.robot_pose.pose.position.z = .75
    sm.userdata.robot_pose.pose.orientation = quaternion_from_euler(1.5708, 0, 1.57081)  # test pose 1.5708, 0, 1.57081

    # result = move(-.78, -.72, .4015, 0, 180, 0, "link6")

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SETCALIB', SetCalib(),
                               transitions={'outcome1': 'CAMERAPOSE',
                                            'outcome2': 'FINDCAR'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool'})
        smach.StateMachine.add('CAMERAPOSE', CameraPose(),
                               transitions={'outcome1': 'SETCALIB'},
                               remapping={'tool_in': 'current_tool'})
        smach.StateMachine.add('FINDCAR', FindCar(),
                               transitions={'outcome1': 'MOVEARM'},
                               remapping={'tool_in': 'current_tool'})

        smach.StateMachine.add('MOVEARM', MoveArm(),
                               transitions={'outcome1': 'outcome6'},
                               remapping={'move_pose': 'robot_pose'})

        sm_con = smach.Concurrence(outcomes=['outcome4', 'outcome5'],
                                   default_outcome='outcome4',
                                   input_keys=['con_in'],
                                   output_keys=['con_out'],
                                   outcome_map={'outcome5':
                                                {'GETCARINFO': 'outcome2',
                                                 'LOCATEFLAP': 'outcome1'}})

        with sm_con:
            # add states to the container
            smach.Concurrence.add('GETCARINFO', GetCarInfo())
            smach.Concurrence.add('LOCATEFLAP', LocateFlap())

        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4': 'CON',
                                            'outcome5': 'outcome6'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    main()
