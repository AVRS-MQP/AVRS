import roslib; roslib.load_manifest('smach_machine')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from actionlib import *
from actionlib_msgs.msg import *
from smach import Concurrence


# define state set_calib
class SetCalib(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['camera_pose'],
                             input_keys=['start', 'current_tool'],
                             output_keys=['current_tool'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SetCalib')
        if userdata.tooling == 1:
            # calls calibrate service based on current tool
            return 'camera_pose'


# define state camera_pose
class CameraPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['find_car'],
                             input_keys=['current_tool'],
                             output_keys=['current_tool'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CameraPose')
        # moves arm to camera pose
        return 'find_car'


# define state find_car
class FindCar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['get_car_info', 'locate_flap'],
                             input_keys=['current_tool'],
                             output_keys=['current_tool', 'has_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state find_car')
        # locates quarter panel only then gets more info
        return 'get_car_info', 'locate_flap'


# define concurrent state get_car_info
class GetCarInfo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             output_keys=['car_model'])

    def execute(self, userdata):
        rospy.loginfo('Executing state get_car_info')
        # gets vehicle info packet from Yun
        return 'succeeded'


# define concurrent state locate_flap
class LocateFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             output_keys=['has_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state get_car_info')
        # CV and point cloud things to get the pose of gas flap
        return 'succeeded'


# define state flap
class Flap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['flap_approach_pose', 'find_port_pose'],
                             input_keys=['car_model'],
                             output_keys=['flap_is_open'])

    def execute(self, userdata):
        rospy.loginfo('Executing state get_car_info')
        if userdata.vehicle_model == 1:
            # Tesla, so port is pre-opened
            # get correct charger and move to pose to scan for port
            return 'find_port_pose'
        else:
            # other model, so gas flap needs to be opened
            return 'flap_approach_pose'


# define state find_port_pose
class FindPortPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['port_approach_pose'],
                             input_keys=['input1'],
                             output_keys=['output1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state find_port_pose')
        # gets pose of port
        return 'outcome1'


# define state flap_approach_pose
class FlapApproachPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['approach_flap'],
                             input_keys=['input1'],
                             output_keys=['output1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state flap_approach_pose')
        # using flap pose gets into correct location and verifies
        return 'outcome1'


# define state find_port_pose
class ApproachFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['open_flap'],
                             input_keys=['input1'],
                             output_keys=['output1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state approach_flap')
        # approaches flap while turing on pneumatic cup
        # successful when vacuum sensor detects suction
        return 'open_flap'


# define state open_flap
class OpenFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['change_tool'],
                             input_keys=['flap_unlocked'],
                             output_keys=['open_error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state get_car_info')
        # must take input that flap is unlocked before acting

        if userdata.flap_unlocked:
            # will then pull back and open flap, turning off pnematics
            # turns off
            return 'change_tool'
        else:
            return 'outcome2'


# define state change_tool
class ChangeTool(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['find_port_pose'],
                             input_keys=['car_model', 'current_tool'],
                             output_keys=['current_tool'])

    def execute(self, userdata):
        rospy.loginfo('Executing state get_car_info')
        # must take input that flap is unlocked before acting

        if userdata.flap_unlocked:
            # puts down current tool and picks up correct one
            return 'find_port_pose'


# define state port_approach_pose
class PortApproachPose(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['approach_port'],
                             input_keys=['input1'],
                             output_keys=['output1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state flap_approach_pose')
        # using port pose gets into correct location and verifies
        return 'outcome1'


# define state approach_port
class ApproachPort(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['charging', 'PortApproachPose'],
                             input_keys=['input1'],
                             output_keys=['output1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state approach_flap')
        if userdata.vehicle_model == 1:  # forces being monitored all along to abort if needed
            # approaches port while monitoring forces
            return 'charging'
        else:
            return 'PortApproachPose'


# define state charging
class Charging(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['change_tool'],
                             input_keys=['battery_charged'])

    def execute(self, userdata):
        rospy.loginfo('Executing state approach_flap')
        # should monitor forces
        # unplugs tool when given signal
        return 'change_tool'


# define state close_flap
class CloseFlap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['start'],
                             input_keys=['current_tool'])

    def execute(self, userdata):
        rospy.loginfo('Executing state approach_flap')
        # should monitor forces
        # unplugs tool when given signal
        return 'change_tool'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome5'])
    sm.userdata.start_signal = 0  # when high sm starts
    sm.userdata.tooling = 0  # 0 no tool, 1 cup, 2 Tesla, 3 J1772, 4 CHAdeMO
    sm.userdata.is_calibrated = 0  # 0: not calibrated 1:calib for current tool
    sm.userdata.flap_open = 0  # 0:flap is closed
    sm.userdata.flap_unlocked = 0  # 1: flap is unlocked
    sm.userdata.vehicle_model = 0
    sm.userdata.pose_found = 0  # 1:recieved pose
    sm.userdata.flap_error = 0  # 1 if coms say open but force on open too high

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('set_calib', SetCalib(),
                               transitions={'camera_pose': 'camera_pose'},
                               remapping={'start': 'start_signal', 'current_tool': 'tooling'})

        smach.StateMachine.add('camera_pose', CameraPose(),
                               transitions={'find_car': 'find_car'},
                               remapping={'current_tool': 'tooling'})

        # THIS SYNTAX IS INCORRECT, VERIFY WITH EXECUTIVE TUTORIALS
        sm_FindCar = smach.Concurrence(
                                outcomes=['open_flap'],
                                default_outcome='open_flap',
                                output_keys=['car_model', 'has_pose'],
                                outcome_map={'succeeded': {'get_car_info': 'succeeded', 'locate_flap': 'succeeded'}})

        # open concurrent sub-container
        with sm_FindCar:
            # Add states
            Concurrence.add('get_car_info', GetCarInfo(),
                            remapping={'car_model': 'vehicle_model'})
            Concurrence.add('locate_flap', LocateFlap(),
                            remapping={'has_pose': 'pose_found'})

        smach.StateMachine.add('find_car', FindCar(),
                               transitions={'get_car_info': 'get_car_info', 'locate_flap': 'locate_flap'},
                               remapping={'current_tool': 'tooling', 'has_pose': 'pose_found'})

        smach.StateMachine.add('flap', Flap(),
                               transitions={'flap_approach_pose': 'flap_approach_pose', 'find_port_pose': 'find_port_pose'},
                               remapping={'car_model': 'vehicle_model', 'flap_is_open': 'flap_open'})

        smach.StateMachine.add('find_port_pose', FindPortPose(),
                               transitions={'port_approach_pose': 'port_approach_pose'},
                               remapping={'': '', '': ''})

        smach.StateMachine.add('flap_approach_pose', FlapApproachPose(),
                               transitions={'approach_flap': 'approach_flap'},
                               remapping={'': '', '': ''})

        smach.StateMachine.add('approach_flap', approach_flap(),
                               transitions={'open_flap': 'open_flap'},
                               remapping={'': '', '': ''})

        smach.StateMachine.add('open_flap', OpenFlap(),
                               transitions={'change_tool': 'change_tool'},
                               remapping={'flap_unlocked': 'flap_unlocked', 'open_error': 'flap_error'})

        smach.StateMachine.add('change_tool', ChangeTool(),
                               transitions={'find_port_pose': 'find_port_pose'},
                               remapping={'car_model': 'vehicle_model', 'current_tool': 'tooling'})

        smach.StateMachine.add('port_approach_pose', PortApproachPose(),
                               transitions={'approach_port': 'approach_port'},
                               remapping={'': '', '': ''})

        smach.StateMachine.add('approach_port', ApproachPort(),
                               transitions={'charging': 'charging'},
                               remapping={'': '', '': ''})

        smach.StateMachine.add('charging', Charging(),
                               transitions={'change_tool': 'change_tool'},
                               remapping={'battery_charged': 'end_charge'})

        smach.StateMachine.add('close_flap', CloseFlap(),
                               transitions={'start': 'start'},
                               remapping={'current_tool': 'tooling'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

# generic state template to copy
# class genericState(smach.Sate):
#    def __init__(self):
#        smach.State.__init__(self,
#                             outcomes=['outcome1'],
#                             input_keys=['input1'],
#                             output_keys=['output1'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state get_car_info')
#        return 'outcome1'
