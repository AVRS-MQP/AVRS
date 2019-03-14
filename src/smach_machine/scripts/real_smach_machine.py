#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

# from motion_functions import *
import actionlib_tutorials_msgs

import motion_msgs
import coms_msgs
from motion_msgs.msg import *
from coms_msgs.msg import *
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
import mode_msgs
from mode_msgs.srv import Mode  #
from cv_service_msgs.srv import cv_service

from geometry_msgs.msg import PoseStamped

from tf import TransformListener  # Autofills to this, and uses it in line 43 so why is it error flagged???
from std_msgs.msg import Float32


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


def remove_tool(current_tool):
    tool = "fsp"  # the control tool

    if current_tool == "vac":
        touching = "h_tool_vac"
        clearing = "h_tool_vac_clear"
        clearing_C = "h_tool_vac_clear_C"

    elif current_tool == "tes":
        touching = "h_tool_tes"
        clearing = "h_tool_tes_clear"
        clearing_C = "h_tool_tes_clear_C"

    elif current_tool == "j17":
        touching = "h_tool_j17"
        clearing = "h_tool_j17_clear"
        clearing_C = "h_tool_j17_clear_C"

    else:
        print("ERROR: Invalid current tool")

    motion_done = False
    # go to clearance
    print("Going to clearance for tool: ", current_tool)

    while not motion_done:
        motion_done = move_target(clearing, tool, 1)
        print("STATUS:", motion_done)
        rospy.sleep(3)
    motion_done = False

    print("Going to clearance_C for tool: ", current_tool)

    while not motion_done:
        motion_done = move_target(clearing_C, tool, 1)
        print("STATUS:", motion_done)
        rospy.sleep(3)
    motion_done = False

    print("wating for abort...")
    rospy.sleep(5)

    # go to holster
    print("Going to dropoff for tool: ", current_tool)

    while not motion_done:
        motion_done = move_target(touching, tool, 1)
        print("STATUS:", motion_done)
        rospy.sleep(3)
    motion_done = False

    print("---REMOVE THE TOOL---")
    rospy.sleep(5)
    # go to clearance
    print("Going to clearance for tool: ", current_tool)

    while not motion_done:
        motion_done = move_target(clearing, tool, 1)
        print("STATUS:", motion_done)
        rospy.sleep(3)
    motion_done = False


def get_tool(new_tool):
    tool = "fsp"  # the control tool

    if new_tool == "vac":
        touching = "h_tool_vac"
        clearing = "h_tool_vac_clear"
        clearing_C = "h_tool_vac_clear_C"

    elif new_tool == "tes":
        touching = "h_tool_tes"
        clearing = "h_tool_tes_clear"
        clearing_C = "h_tool_tes_clear_C"

    elif new_tool == "j17":
        touching = "h_tool_j17"
        clearing = "h_tool_j17_clear"
        clearing_C = "h_tool_j17_clear_C"

    else:
        print("ERROR: Invalid current tool")

    motion_done = False

    # go to clearance
    print("Going to clearance for tool: ", new_tool)
    # (trans, rot) = get_pose_from_tf("base_link", clearing)
    # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)
    # client.send_goal(goal)  # Sends the goal to the action server.
    # print("Waiting for server")
    # client.wait_for_server()
    while not motion_done:
        motion_done = move_target(clearing, tool, 1)
        print("STATUS:", motion_done)
    motion_done = False

    while not motion_done:
        motion_done = move_target(clearing_C, tool, 1)
        print("STATUS:", motion_done)
    motion_done = False

    # go to holster
    print("Going to pickup for tool: ", new_tool)
    # (trans, rot) = get_pose_from_tf("base_link", touching)
    # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    # client.send_goal(goal)  # Sends the goal to the action server.
    # print("Waiting for server")
    # client.wait_for_server()
    while not motion_done:
        motion_done = move_target(touching, tool, 1)
        print("STATUS:", motion_done)
    motion_done = False

    print("---GET THE TOOL---")
    rospy.sleep(10)
    # go to clearance
    print("Going to clearance for tool: ", new_tool)
    # (trans, rot) = get_pose_from_tf("base_link", clearing)
    # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    # client.send_goal(goal)  # Sends the goal to the action server.
    # print("Waiting for server")
    # client.wait_for_server()
    while not motion_done:
        motion_done = move_target(clearing, tool, 1)
        print("STATUS:", motion_done)
    motion_done = False


def move_target(target_frame, tool, mode):
    client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)
    client.wait_for_server()

    (trans, rot) = get_pose_from_tf("base_link", target_frame)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, mode)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_result()

    return client.get_result()


def move_simple(x, y, z, t, u, v, w, tool, mode):
    client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)
    client.wait_for_server()

    goal = motion_msgs.msg.MoveRobotQuatGoal(x, y, z, t, u, v, w, tool, mode)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_result()

    return client.get_result()


def move_zero():
    client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)
    client.wait_for_server()

    goal = motion_msgs.msg.MoveRobotQuatGoal(0, 0, 0, 0, 0, 0, 0, 'na', 0)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_result()

    return client.get_result()


# note: Reference States
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
        # output = PoseStamped()

        # output.pose.position = userdata.move_pose.pose.position
        # output.pose.orientation = userdata.move_pose.pose.orientation

        # TEST
        (trans, rot) = get_pose_from_tf("base_link", "flap_clearance")

        # THIS IS HOW TO PULL XYZRPY FROM POSE
        # print("Pose")
        # print(userdata.move_pose)

        # print("OUTPUT", output)
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)

        print(userdata.move_pose)

        # Sends the goal to the action server.
        client.send_goal(goal)

        return 'outcome1'


# note: State Machine start
# define state SetCalib
# based on current tool, calibrates force sensors and moves into the camera pose
class FindVehicle(smach.State):
    def coms_callback(self, data):
        self.car_model = data.model
        self.charger = data.charger_type
        self.battery_pcnt = data.battery_charge
        self.charge_lvl = data.charge_level

        print("in Coms CB")

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['vehicle_found', 'still_searching'],
                             output_keys=[])

        # temp variable, recursion forever using just userdata
        self.counter = 0
        self.yun_coms = rospy.Subscriber('comsUplink', Vehicle, self.coms_callback)

    def execute(self, userdata):
        rospy.loginfo('Executing state GetCarInfo')
        if self.counter < 2:
            self.counter = self.counter + 1
            return 'still_searching'
        else:
            return 'vehicle_found'


# define state Find2DFlap
# CV get x,y location of the flap
class Find2DFlap(smach.State):
    x_camera = -1
    y_camera = -1

    def cv_callback(self, data):
        self.camera_x = data.pose.position.x
        self.camera_y = data.pose.position.y

        print("in CV CB")

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['correct_pos', 'wrong_pos'])

        self.cv_sub = rospy.Subscriber('cv', PoseStamped, self.cv_callback)

    def in_tolerance(self, value, target, range):
        if value < target + range and value > target - range:
            return True
        else:
            return False

    def execute(self, userdata):

        motion_client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)
        cv_client = rospy.ServiceProxy('cv_service', cv_service)
        tf_man = rospy.ServiceProxy('transform', Mode)

        done = False
        doneX = False
        doneY = False
        tolerance = .005
        target = .5
        kp = .2
        motion_done = False

        tool = "cam"

        motion_done = False

        print("moving to loc_PI6")
        while not motion_done:
            motion_done = move_target("loc_PI6", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False

        print("moving to loc_PI3")
        while not motion_done:
            motion_done = move_target("loc_PI3", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False

        print("moving to loc_B")
        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False

        print("moving to loc_C")
        while not motion_done:
            motion_done = move_target("loc_C", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False

        print("at loc_C")

        rospy.sleep(5)

        while not rospy.is_shutdown() and not done:
            print("zeroing on target (doneX,doneY)", doneX, doneY)

            mode = 1
            responce = cv_client(mode)  # request info

            print("CV result:", responce)

            if self.in_tolerance(responce.flapX, target, tolerance):
                doneX = True
                print("X Zeroed")

            if not doneX:
                diffX = responce.flapX - target
                dx = diffX * kp * -1
                print("dx", dx)
            else:
                dx = 0

            if (self.in_tolerance(responce.flapY, target, tolerance)):
                doneY = True
                print("Y Zeroed")

            if not doneY:
                diffY = responce.flapY - target
                dy = diffY * kp * -1
                print("dy", dy)
            else:
                dy = 0

            move_simple(dx, dy, 0, 0, 0, 0, 1, tool, 7)
            # goal = motion_msgs.msg.MoveRobotQuatGoal(dx, dy, 0, 0, 0, 0, 1, tool, 7)
            # motion_client.send_goal(goal)
            # motion_client.wait_for_server()
            rospy.sleep(.2)

            if doneX and doneY:
                done = True
                print("fully zeroed")

            if done:
                print("saving camera location")
                try:

                    tf_man("save_singular", 3)
                    rospy.sleep(4)
                    print("publishing...")
                    tf_man("publish_singular", 4)
                    rospy.sleep(4)

                    print("Tried and succeeded")

                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
                    print("Tried and failed")

                print("moving camera out of the way of point cloud")
                print("moving to loc_B")
                while not motion_done:
                    motion_done = move_target("loc_B", tool, 1)
                    print("STATUS:", motion_done)

                rospy.sleep(5)

        rospy.sleep(12)

        return 'correct_pos'


# define state Find3DFlap
# PC get pose of flap based on CV x,y loc
class Find3DFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['flap_pose_saved'])
        self.tf = TransformListener()

    def execute(self, userdata):
        print("Waiting for fusion srv")

        rospy.wait_for_service('fusion')

        try:
            fuser = rospy.ServiceProxy('fusion', Mode)
            tf_man = rospy.ServiceProxy('transform', Mode)

            fuser("find", 1)
            print("finding...")
            rospy.sleep(6)
            print("saving...")
            tf_man("save", 1)
            rospy.sleep(4)
            print("publishing...")
            tf_man("publish", 2)  # change if cam save is needed for debug
            rospy.sleep(4)
            print("stopping fusion")
            fuser("stop", 0)  # change if cam save is needed for debug
            print("Tried and succeeded")

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            print("Tried and failed")

        return 'flap_pose_saved'


# define state OpenFlap
class OpenFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenFlap')
        print("Moving to flap")

        client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

        print("Waiting for server")
        client.wait_for_server()

        hingePub.publish(0)
        rospy.sleep(2)

        # go to clearance
        tool = "vac"

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_clearance", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_touching", tool, 1)
            print("STATUS:", motion_done)
        motion_done = False

        print("SUCK")
        rospy.sleep(10)

        # open the flap
        for i in range(0, 70, 5):
            rospy.sleep(1)
            hingePub.publish(i)
            while not motion_done:
                motion_done = move_target("flap_touching", tool, 1)
            motion_done = False

            print("STATUS:", motion_done)

            print("Waiting for server")

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_clearance2", tool, 1)

        rospy.sleep(5)
        tool = "cam"

        motion_done = False
        while not motion_done:
            motion_done = move_target("loc_C", tool, 1)
        motion_done = False
        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)

        rospy.sleep(6)
        return 'outcome1'


# define state ChangeToolCharger
class ChangeToolCharger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['tool_changed'],
                             input_keys=['tool_in', 'charging_port'],
                             output_keys=['tool_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeToolCharger')
        rospy.loginfo(userdata.tool_in)

        motion_done = False
        charger_type = userdata.charging_port  # "j17"  # needs to be set to "tes" or "j17" todo

        # got to pose to avoid singularity
        tool = "fsp"  # moving from tool change for swapping tools
        print("avoiding singularity")

        print("moving to vac_clear")
        while not motion_done:
            motion_done = move_target("h_tool_vac_clear", tool, 1)
            print("STATUS:", motion_done)
        rospy.sleep(3)
        motion_done = False

        remove_tool("vac")  # return tool
        get_tool(charger_type)  # get new tool

        userdata.tool_out = charger_type

        tool = "cam"

        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)
            print("STATUS:", motion_done)
        rospy.sleep(3)
        motion_done = False

        return 'tool_changed'


# # define state PlugIn
class PlugIn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'])

    def in_tolerance(self, value, target, range):
        if value < target + range and value > target - range:
            return True
        else:
            return False

    def execute(self, userdata):
        rospy.loginfo('Executing state PlugIn')
        cv_client = rospy.ServiceProxy('cv_service', cv_service)
        tf_man = rospy.ServiceProxy('transform', Mode)
        motion_done = False

        done = False
        doneX = False
        doneY = False
        tolerance = .005
        target = .5
        kp = .2

        tool = "cam"

        tf_man("publish_flap", 2)  # publish the saved flap

        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)
            print("STATUS:", motion_done)
        motion_done = False

        print("at loc_B")

        rospy.sleep(5)
        while not motion_done:
            motion_done = move_target("cam_clearance", tool, 1)
            print("STATUS:", motion_done)
            rospy.sleep(3)
        motion_done = False

        print("at flap_clearance")

        rospy.sleep(5)

        while not rospy.is_shutdown() and not done:
            print("zeroing on target (doneX,doneY)", doneX, doneY)

            cv_mode = 1
            responce = cv_client(cv_mode)  # request info TODO add mode for port searching

            print("CV result:", responce)

            if (self.in_tolerance(responce.flapX, target, tolerance)):
                doneX = True
                print("X Zeroed")

            if not doneX:
                diffX = responce.flapX - target
                dx = diffX * kp * -1
                print("dx", dx)
            else:
                dx = 0

            if (self.in_tolerance(responce.flapY, target, tolerance)):
                doneY = True
                print("Y Zeroed")

            if not doneY:
                diffY = responce.flapY - target
                dy = diffY * kp * -1
                print("dy", dy)
            else:
                dy = 0

            while not motion_done:
                    motion_done = move_simple(dx, dy, 0, 0, 0, 0, 1, tool, 7)
                    print("STATUS:", motion_done)
                    rospy.sleep(3)
            motion_done = False

            rospy.sleep(.2)

            if doneX and doneY:
                done = True
                print("fully zeroed")

            if done:
                print("saving camera location")
                try:

                    tf_man("save_singular", 3)
                    rospy.sleep(4)
                    print("publishing...")
                    tf_man("publish_singular", 4)
                    rospy.sleep(4)

                    print("Tried and succeeded")

                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
                    print("Tried and failed")

                tool = 'j17'  # TODO hardcoded for testing

                if tool == 'j17':
                    mode = 6
                elif tool == 'tes':
                    mode = 5
                else:
                    print("Invalid tool for plugin")

                print("Plugging in the charger")
                while not motion_done:
                    motion_done = move_target("cam_saved", tool, 1)
                    print("STATUS:", motion_done)
                    rospy.sleep(5)
                motion_done = False

                dz = .18

                while not motion_done:
                    motion_done = move_simple(0, 0, dz, 0, 0, 0, 1, tool, mode)
                    print("STATUS:", motion_done)
                    rospy.sleep(3)
                motion_done = False

                rospy.sleep(2)

        # # todo remove test only
        # tool = "vac"
        #
        # print("moving camera to see charger")
        # (trans, rot) = get_pose_from_tf("base_link",
        #                                 "arm_cam_pose")  # todo should be flap_hole_clearance, setting to other for testing
        # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        # client.send_goal(goal)
        # client.wait_for_server()
        # rospy.sleep(10)
        #
        # # trying to do a tool frame motion
        # goal = motion_msgs.msg.MoveRobotQuatGoal(0, 0, .5, 0, 0, 0, 1, tool, 3)
        # client.send_goal(goal)
        # client.wait_for_server()
        # rospy.sleep(10)
        #
        # print("moving camera to see charger")
        # (trans, rot) = get_pose_from_tf("base_link",
        #                                 "arm_cam_pose")  # todo should be flap_hole_clearance, setting to other for testing
        # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        # client.send_goal(goal)
        # client.wait_for_server()
        # rospy.sleep(10)
        #
        # # trying to do a tool frame motion
        # goal = motion_msgs.msg.MoveRobotQuatGoal(0, .2, 0, 0, 0, 0, 1, tool, 3)
        # client.send_goal(goal)
        # client.wait_for_server()
        # rospy.sleep(10)

        # todo end test code

        return 'outcome1'


# define state ChangeToolCharger
class ChangeToolVac(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['tool_changed'],
                             input_keys=['tool_in', 'charging_port'],
                             output_keys=['tool_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeToolCharger')
        rospy.loginfo(userdata.tool_in)

        motion_done = False
        charger_type = userdata.charging_port  # "j17"  # needs to be set to "tes" or "j17" todo

        # got to pose to avoid singularity
        tool = "fsp"  # moving from tool change for swapping tools
        print("avoiding singularity")

        print("moving to vac_clear")
        while not motion_done:
            motion_done = move_target("h_tool_vac_clear", tool, 1)
            print("STATUS:", motion_done)
        rospy.sleep(3)
        motion_done = False

        remove_tool(charger_type)  # return tool
        get_tool("vac")  # get new tool

        userdata.tool_out = "vac"

        tool = "cam"

        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)
            print("STATUS:", motion_done)
        rospy.sleep(3)
        motion_done = False

        return 'tool_changed'


class CloseFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['flap_closed'])

    def execute(self, userdata):
        print("-----CloseFlap-----\n")

        hingePub.publish(70)  # todo pass data on hinge angle around
        rospy.sleep(2)

        # go to clearance
        tool = "vac"

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_clearance2", tool, 1)
            print("STATUS:", motion_done)

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_clearance_C", tool, 1)
            print("STATUS:", motion_done)
        motion_done = False

        print("SUCK")
        rospy.sleep(10)

        # open the flap
        for i in range(70, 0, -5):
            rospy.sleep(1)
            hingePub.publish(i)
            while not motion_done:
                motion_done = move_target("flap_touching", tool, 1)
            motion_done = False

            print("STATUS:", motion_done)

            print("Waiting for server")

        motion_done = False
        while not motion_done:
            motion_done = move_target("flap_clearance", tool, 1)

        rospy.sleep(5)
        tool = "cam"

        motion_done = False
        while not motion_done:
            motion_done = move_target("loc_B", tool, 1)
            rospy.sleep(3)
        motion_done = False

        while not motion_done:
            motion_done=move_zero()
            rospy.sleep(3)
        motion_done = False

        rospy.sleep(6)

        return 'flap_closed'


# note: Unused States

# define state GetCarInfo
# CON state, currently unused
class GetCarInfo(smach.State):
    def coms_callback(self, data):
        self.car_model = data.model
        self.charger = data.charger_type
        self.battery_pcnt = data.battery_charge
        self.charge_lvl = data.charge_level

        print("in Coms CB")

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        # temp variable, recursion forever using just userdata
        self.counter = 0
        self.yun_coms = rospy.Subscriber('comsUplink', Vehicle, self.coms_callback)

    def execute(self, userdata):
        rospy.loginfo('Executing state GetCarInfo')
        if self.counter < 2:
            userdata.con_out = userdata.con_in + 1
            self.counter = self.counter + 1
            return 'outcome1'
        else:
            return 'outcome2'


def main():
    rospy.init_node('smach_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXITSMACH'])
    sm.userdata.current_tool = "vac"  # TODO  0:tool plate  1:suction cup  2:Tesla  3:J1772
    sm.userdata.charger_type = "unknown"

    # result = move(-.78, -.72, .4015, 0, 180, 0, "link6")

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FINDVEHICLE', FindVehicle(),
                               transitions={'vehicle_found': 'FIND2DFLAP',  # note should be FIND2DFLAP
                                            'still_searching': 'FINDVEHICLE'},
                               remapping={'charging_port': 'charger_type'})

        smach.StateMachine.add('FIND2DFLAP', Find2DFlap(),
                               transitions={'correct_pos': 'FIND3DFLAP',  # note should be FIND3DFLAP
                                            'wrong_pos': 'FIND2DFLAP'})

        smach.StateMachine.add('FIND3DFLAP', Find3DFlap(),
                               transitions={'flap_pose_saved': 'OPENFLAP'})

        smach.StateMachine.add('OPENFLAP', OpenFlap(),
                               transitions={'outcome1': 'CHANGETOOLCHARGER'})

        smach.StateMachine.add('CHANGETOOLCHARGER', ChangeToolCharger(),
                               transitions={'tool_changed': 'PLUGIN'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool',
                                          'charging_port': 'charger_type'})

        smach.StateMachine.add('PLUGIN', PlugIn(),
                               transitions={'outcome1': 'CHANGETOOLVAC'})

        smach.StateMachine.add('CHANGETOOLVAC', ChangeToolVac(),
                               transitions={'tool_changed': 'CLOSEFLAP'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool',
                                          'charging_port': 'charger_type'})

        smach.StateMachine.add('CLOSEFLAP', CloseFlap(),
                               transitions={'flap_closed': 'EXITSMACH'})
        # note: Reference States
        # smach.StateMachine.add('MOVEARM', MoveArm(),
        #                        transitions={'outcome1': 'EXITSMACH'},
        #                        remapping={'move_pose': 'robot_pose'})

        # note: Unused States
        # smach.StateMachine.add('FINDCAR', FindCar(),
        #                        transitions={'outcome1': 'OPENFLAP'},  # Change 'CON' to 'OPENFLAP' to skip concurrence
        #                        remapping={'tool_in': 'current_tool'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    while not rospy.is_shutdown():
        rospy.spin()
    sis.stop()


if __name__ == '__main__':
    hingePub = rospy.Publisher("hinge_angle", Float32, queue_size=10)
    main()
