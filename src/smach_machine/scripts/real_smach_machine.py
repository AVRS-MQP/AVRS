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

from tf import TransformListener


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

        tool = "vac"
        output = PoseStamped()

        output.pose.position = userdata.move_pose.pose.position
        output.pose.orientation = userdata.move_pose.pose.orientation

        # TEST
        (trans, rot) = get_pose_from_tf("base_link", "flap_raw")

        # THIS IS HOW TO PULL XYZRPY FROM POSE
        print("Pose")
        print(userdata.move_pose)

        print("OUTPUT", output)
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)

        # Sends the goal to the action server.
        client.send_goal(goal)

        return 'outcome1'


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
class FindCar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['tool_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CameraPose')
        rospy.loginfo('Counter = %f' % userdata.tool_in)
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


# define state OpenFlap
class OpenFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['move_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenFlap')
        rospy.loginfo(userdata.move_pose)

        userdata.move_pose.pose.position.x = 3
        userdata.move_pose.pose.position.y = 1
        userdata.move_pose.pose.position.z = .5
        userdata.move_pose.pose.orientation = [1, 2, 3, 4]
        return 'outcome1'


# define state ChangeTool
class ChangeTool(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['tool_in'],
                             output_keys=['tool_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenFlap')
        rospy.loginfo(userdata.tool_in)
        return 'outcome1'


# # define state ChargePort
class ChargePort(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['move_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenFlap')

        rospy.loginfo(userdata.move_pose)
        rospy.loginfo('Exiting state machine')
        return 'outcome1'


class Find2DFlap(smach.State):
    x_camera = -1
    y_camera = -1

    def cv_callback(self, data):
        self.camera_x = data.pose.position.x
        self.camera_y = data.pose.position.y

        print("in CV CB")

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['correct_pos', 'wrong_pos'],
                             input_keys=['camera_x', 'camera_y'])
        self.cv_sub = rospy.Subscriber('cv', PoseStamped, self.cv_callback)

    def execute(self, userdata):
        # if userdata.camera_x == .5 and userdata.camera_y == .5:  # camera in correct position

        return 'correct_pos'

        # else:  # camera pos incorrect
        #
        #     userdata.camera_x = .5
        #     userdata.camera_y = .5
        #     return 'wrong_pos'


class Find3DFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['flap_pose_saved'],
                             input_keys=['camera_x', 'camera_y'])
        self.tf = TransformListener()

    def execute(self, userdata):
        activator_pub = rospy.Publisher('point_cloud_mode', std_msgs.msg.String, queue_size=5)

        # mode 0 defult,1 create, 2 save

        while not rospy.is_shutdown():
            activator_pub.publish("find")  # create the flap pose

            try:
                now=rospy.Time.now()
                self.tf.waitForTransform("/base_link","/flap_raw",now,rospy.Duration(3.0))
                self.tf.lookupTransform("/base_link", "/flap_raw",now)

                activator_pub.publish("save")  # save it
                rospy.sleep(6)
                activator_pub.publish("publish")
                return 'flap_pose_saved'
            except(tf.LookupException, tf.ConnectivityException):
                continue
                print("looking for transform")
                rospy.sleep(6)


def main():
    rospy.init_node('smach_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit_smach'])
    sm.userdata.current_tool = 0
    sm.userdata.con_in = 0
    sm.userdata.con_out = 0
    sm.userdata.robot_pose = PoseStamped()

    sm.userdata.robot_pose.pose.position.x = 1.5
    sm.userdata.robot_pose.pose.position.y = .5
    sm.userdata.robot_pose.pose.position.z = .75
    sm.userdata.robot_pose.pose.orientation = quaternion_from_euler(1.5708, 0, 1.57081)  # test pose 1.5708, 0, 1.57081

    sm.userdata.camera_pos_x = -1
    sm.userdata.camera_pos_y = -1

    # result = move(-.78, -.72, .4015, 0, 180, 0, "link6")

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SETCALIB', SetCalib(),
                               transitions={'outcome1': 'SETCALIB',
                                            'outcome2': 'FINDCAR'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool'})

        smach.StateMachine.add('FINDCAR', FindCar(),
                               transitions={'outcome1': 'OPENFLAP'},  # Change 'CON' to 'OPENFLAP' to skip concurrence
                               remapping={'tool_in': 'current_tool'})

        smach.StateMachine.add('MOVEARM', MoveArm(),
                               transitions={'outcome1': 'exit_smach'},
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
                                            'outcome5': 'OPENFLAP'})

        smach.StateMachine.add('OPENFLAP', OpenFlap(),
                               transitions={'outcome1': 'CHANGETOOL'},
                               remapping={'move_pose': 'robot_pose'})
        smach.StateMachine.add('CHANGETOOL', ChangeTool(),
                               transitions={'outcome1': 'CHARGEPORT'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool'})
        smach.StateMachine.add('CHARGEPORT', ChargePort(),
                               transitions={'outcome1': 'FIND2DFLAP'},
                               remapping={'move_pose': 'robot_pose'})

        smach.StateMachine.add('FIND2DFLAP', Find2DFlap(),
                               transitions={'correct_pos': 'FIND3DFLAP',
                                            'wrong_pos': 'FIND2DFLAP'},
                               remapping={'camera_x': 'camera_pos_x',
                                          'camera_y': 'camera_pos_y'})
        smach.StateMachine.add('FIND3DFLAP', Find3DFlap(),
                               transitions={'flap_pose_saved': 'exit_smach'},
                               remapping={'camera_x': 'camera_pos_x',
                                          'camera_y': 'camera_pos_y'})
    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    while not rospy.is_shutdown():
        rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    main()
