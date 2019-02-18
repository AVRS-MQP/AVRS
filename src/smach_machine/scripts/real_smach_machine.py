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
    elif current_tool == "tes":
        touching = "h_tool_vac"
        clearing = "h_tool_vac_clear"
    elif current_tool == "j17":
        touching = "h_tool_j17"
        clearing = "h_tool_j17_clear"
    else:
        print("ERROR: Invalid current tool")

    client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

    # go to clearance
    print("Going to clearance for tool: ", current_tool)
    (trans, rot) = get_pose_from_tf("base_link", clearing)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()

    # go to holster
    print("Going to dropoff for tool: ", current_tool)
    (trans, rot) = get_pose_from_tf("base_link", touching)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()

    print("REMOVE THE TOOL")
    rospy.sleep(6)
    # go to clearance
    print("Going to clearance for tool: ", current_tool)
    (trans, rot) = get_pose_from_tf("base_link", clearing)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()


def get_tool(new_tool):
    tool = "fsp"  # the control tool

    if new_tool == "vac":
        touching = "h_tool_vac"
        clearing = "h_tool_vac_clear"
    elif new_tool == "tes":
        touching = "h_tool_vac"
        clearing = "h_tool_vac_clear"
    elif new_tool == "j17":
        touching = "h_tool_j17"
        clearing = "h_tool_j17_clear"
    else:
        print("ERROR: Invalid current tool")

    client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

    # go to clearance
    print("Going to clearance for tool: ", new_tool)
    (trans, rot) = get_pose_from_tf("base_link", clearing)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()

    # go to holster
    print("Going to dropoff for tool: ", new_tool)
    (trans, rot) = get_pose_from_tf("base_link", touching)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()

    print("GET THE TOOL")
    rospy.sleep(6)
    # go to clearance
    print("Going to clearance for tool: ", new_tool)
    (trans, rot) = get_pose_from_tf("base_link", clearing)
    goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
    client.send_goal(goal)  # Sends the goal to the action server.
    print("Waiting for server")
    client.wait_for_server()


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
class SetCalib(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['sm_ready'],
                             output_keys=['tool_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SetCalib')
        userdata.tool_in = 1  # Suction cup
        print("Moving to Camera Pose")

        client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

        print("Waiting for server")
        client.wait_for_server()

        tool = "vac"

        # TODO if possible wait at each goal send until the arm has completed the motion
        # Move to a valid pose so the arm doesn't try and break itself to get to arm_cam_pose
        # Works fine if tool change pose is used, if that changes the intmd work as well
        (trans, rot) = get_pose_from_tf("base_link", "h_tool_j17_clear")
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)

        # Sends the goal to the action server.
        client.send_goal(goal)
        print("Waiting for result")
        client.wait_for_result()

        # (trans, rot) = get_pose_from_tf("base_link", "intmd_pose_right")
        # goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        #
        # # Sends the goal to the action server.
        # client.send_goal(goal)
        # rospy.sleep(15)

        (trans, rot) = get_pose_from_tf("base_link", "arm_cam_pose")
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)

        # Sends the goal to the action server.
        client.send_goal(goal)
        print("Waiting for result")
        client.wait_for_result()
        return 'sm_ready'


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
                             outcomes=['correct_pos', 'wrong_pos'],
                             input_keys=['camera_x', 'camera_y'])
        self.cv_sub = rospy.Subscriber('cv', PoseStamped, self.cv_callback)

    def in_tolerance(self,value,target,range):
        if value<target+range or value>target-range:
            return True
        else:
            return False

    def execute(self, userdata):
        # if userdata.camera_x == .5 and userdata.camera_y == .5:  # camera in correct position
        print("Waiting for CV srv")


        rospy.wait_for_service('motion')
        rospy.wait_for_service('cv_service')

        client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)
        cv = rospy.ServiceProxy('cv_service',cv_service)

        done=False
        doneX=False
        doneY=False
        tolerance=.05



        #todo remove test only
        tool = "vac"

        print("moving camera to see charger")
        (trans, rot) = get_pose_from_tf("base_link", "arm_cam_pose")# todo should be flap_hole_clearance, setting to other for testing
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        # trying to do a tool frame motion
        goal = motion_msgs.msg.MoveRobotQuatGoal(0,0,.5, 0, 0, 0, 1, tool, 4)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        print("moving camera to see charger")
        (trans, rot) = get_pose_from_tf("base_link", "arm_cam_pose")# todo should be flap_hole_clearance, setting to other for testing
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        # trying to do a tool frame motion
        goal = motion_msgs.msg.MoveRobotQuatGoal(0, .2, 0, 0, 0, 0, 1, tool, 4)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        #todo end test code


        #FUTURE ZEROING CODE

        while not rospy.is_shutdown() and not done:
            mode=1
            responce=cv(mode)

            print("result",responce)
            rospy.sleep(2)

            if not doneX:

                if(self.in_tolerance(responce.flapX,.5,tolerance)):
                    doneX=True


            if(doneX and doneY):
                done=True

        rospy.wait_for_service('cv_pose')  #TODO match with Matt's CV service update service to tage in args for flap, j17, tes
        cv_flap = rospy.ServiceProxy('cv_pose', Mode)

        cv_flap("find", 1)
        rospy.sleep(4)

        return 'correct_pos'

        # else:  # camera pos incorrect
        #
        #     userdata.camera_x = .5
        #     userdata.camera_y = .5
        #     return 'wrong_pos'


# define state Find3DFlap
# PC get pose of flap based on CV x,y loc
class Find3DFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['flap_pose_saved'],
                             input_keys=['camera_x', 'camera_y'])
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
            tf_man("publish", 2)
            rospy.sleep(4)
            print("stopping fusion")
            fuser("stop", 0)

            print("Tried and succeeded")

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            print("Tried and failed")

        return 'flap_pose_saved'
        # else:  #will run after except if no errors were raised

        # activator_pub = rospy.Publisher('point_cloud_mode', std_msgs.msg.String, queue_size=5)

        # # mode 0 defult,1 create, 2 save
        #
        # while not rospy.is_shutdown():
        #     activator_pub.publish("find")  # create the flap pose
        #
        #     try:
        #         now=rospy.Time.now()
        #         self.tf.waitForTransform("/base_link","/flap_raw",now,rospy.Duration(3.0))
        #         self.tf.lookupTransform("/base_link", "/flap_raw",now)
        #
        #         activator_pub.publish("save")  # save it
        #         rospy.sleep(6)
        #         activator_pub.publish("publish")
        #         return 'flap_pose_saved'
        #     except(tf.LookupException, tf.ConnectivityException):
        #         continue
        #         print("looking for transform")
        #         rospy.sleep(6)


# define state OpenFlap
# move to
class OpenFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['move_pose'])

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
        (trans, rot) = get_pose_from_tf("base_link", "flap_clearance")
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 1)
        client.send_goal(goal)  # Sends the goal to the action server.
        print("Waiting for server")
        client.wait_for_server()

        # got to touching
        rospy.sleep(6)
        (trans, rot) = get_pose_from_tf("base_link", "flap_touching")
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)  # Sends the goal to the action server.
        print("Waiting for server")
        client.wait_for_server()

        print("SUCK")
        rospy.sleep(10)

        # open the flap
        for i in range(0, 80, 3):
            rospy.sleep(1)
            hingePub.publish(i)
            (trans, rot) = get_pose_from_tf("base_link", "flap_touching")
            goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool,
                                                     2)
            client.send_goal(goal)  # Sends the goal to the action server.
            print("Waiting for server")
            client.wait_for_server()

        # once open go back to clearance
        (trans, rot) = get_pose_from_tf("base_link", "flap_clearance2")
        #todo need to update flap_clearance2 inside tf_man when we get to test with point cloud again
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()

        rospy.sleep(6)
        return 'outcome1'


# define state ChangeToolCharger
class ChangeToolCharger(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['tool_changed'],
                             input_keys=['tool_in'],
                             output_keys=['tool_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeToolCharger')
        rospy.loginfo(userdata.tool_in)

        client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

        charger_type="j17"#needs to be set to "tes" or "j17" todo

        remove_tool("vac")
        get_tool(charger_type)



        #got to pose to avoid singularity
        tool="cam"
        print("avoiding singulairty")
        (trans, rot) = get_pose_from_tf("base_link", "intmd_pose_left")
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(1)







        userdata.tool_out = 2




        return 'tool_changed'


# # define state PlugIn
class PlugIn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['move_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlugIn')
        client = actionlib.SimpleActionClient('motion', motion_msgs.msg.MoveRobotQuatAction)

                 #todo remove test only
        tool = "vac"

        print("moving camera to see charger")
        (trans, rot) = get_pose_from_tf("base_link", "arm_cam_pose")# todo should be flap_hole_clearance, setting to other for testing
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        # trying to do a tool frame motion
        goal = motion_msgs.msg.MoveRobotQuatGoal(0,0,.5, 0, 0, 0, 1, tool, 3)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        print("moving camera to see charger")
        (trans, rot) = get_pose_from_tf("base_link", "arm_cam_pose")# todo should be flap_hole_clearance, setting to other for testing
        goal = motion_msgs.msg.MoveRobotQuatGoal(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], tool, 2)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        # trying to do a tool frame motion
        goal = motion_msgs.msg.MoveRobotQuatGoal(0, .2, 0, 0, 0, 0, 1, tool, 3)
        client.send_goal(goal)
        client.wait_for_server()
        rospy.sleep(10)

        #todo end test code

        return 'outcome1'


# note: Unused States
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
# CON state, currently unused
class GetCarInfo(smach.State):
    def coms_callback(self, data):
        self.car_model = data.model
        self.charger = data.charger_type
        self.battery_pcnt = data.battery_charge
        self.charge_lvl = data.charge_level

        print("in Coms CB")

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'],
                             input_keys=['con_in'],
                             output_keys=['con_out'])
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


# define state LocateFlap
# CON state, currently unused
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
    sm = smach.StateMachine(outcomes=['EXITSMACH'])
    sm.userdata.current_tool = 1  # TODO  0:tool plate  1:suction cup  2:Tesla  3:J1772
    sm.userdata.con_in = 0
    sm.userdata.con_out = 0
    sm.userdata.robot_pose = PoseStamped()

    sm.userdata.robot_pose.pose.position.x = 2
    sm.userdata.robot_pose.pose.position.y = .5
    sm.userdata.robot_pose.pose.position.z = -.75
    sm.userdata.robot_pose.pose.orientation = quaternion_from_euler(1.57081, 0,
                                                                    1.57081)  # test pose 1.57081, 0, 1.57081

    sm.userdata.camera_pos_x = -1
    sm.userdata.camera_pos_y = -1

    # result = move(-.78, -.72, .4015, 0, 180, 0, "link6")

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SETCALIB', SetCalib(),
                               transitions={'sm_ready': 'FIND2DFLAP'}, ## should be FIND2DFLAP
                               remapping={'tool_in': 'current_tool'})

        smach.StateMachine.add('FIND2DFLAP', Find2DFlap(),
                               transitions={'correct_pos': 'CHANGETOOLCHARGER',## should be FIND3DFLAP
                                            'wrong_pos': 'FIND2DFLAP'},
                               remapping={'camera_x': 'camera_pos_x',
                                          'camera_y': 'camera_pos_y'})

        smach.StateMachine.add('FIND3DFLAP', Find3DFlap(),
                               transitions={'flap_pose_saved': 'OPENFLAP'},
                               remapping={'camera_x': 'camera_pos_x',
                                          'camera_y': 'camera_pos_y'})

        smach.StateMachine.add('OPENFLAP', OpenFlap(),
                               transitions={'outcome1': 'CHANGETOOLCHARGER'},
                               remapping={'move_pose': 'robot_pose'})

        smach.StateMachine.add('CHANGETOOLCHARGER', ChangeToolCharger(),
                               transitions={'tool_changed': 'PLUGIN'},
                               remapping={'tool_in': 'current_tool',
                                          'tool_out': 'current_tool'})

        smach.StateMachine.add('PLUGIN', PlugIn(),
                               transitions={'outcome1': 'EXITSMACH'},
                               remapping={'move_pose': 'robot_pose'})

        # note: Reference States
        # smach.StateMachine.add('MOVEARM', MoveArm(),
        #                        transitions={'outcome1': 'EXITSMACH'},
        #                        remapping={'move_pose': 'robot_pose'})

        # note: Unused States
        # smach.StateMachine.add('FINDCAR', FindCar(),
        #                        transitions={'outcome1': 'OPENFLAP'},  # Change 'CON' to 'OPENFLAP' to skip concurrence
        #                        remapping={'tool_in': 'current_tool'})
        #
        # sm_con = smach.Concurrence(outcomes=['outcome4', 'outcome5'],
        #                            default_outcome='outcome4',
        #                            input_keys=['con_in'],
        #                            output_keys=['con_out'],
        #                            outcome_map={'outcome5':
        #                                             {'GETCARINFO': 'outcome2',
        #                                              'LOCATEFLAP': 'outcome1'}})
        #
        # with sm_con:
        #     # add states to the container
        #     smach.Concurrence.add('GETCARINFO', GetCarInfo())
        #     smach.Concurrence.add('LOCATEFLAP', LocateFlap())
        #
        # smach.StateMachine.add('CON', sm_con,
        #                        transitions={'outcome4': 'CON',
        #                                     'outcome5': 'OPENFLAP'})

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
