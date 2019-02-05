
class Robot:

    lastX = 0
    lastY = -.8
    lastZ = .4
    lastRoll = 0
    lastPitch = 180
    lastYaw = 0
    lastTool = 0

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
        # moves arm to a new x,y position without changing z
    
    def moveXZ(self, x, z):
        self.lastX = x
        self.lastZ = z

        self.client.wait_for_server()
        goal = motion_msgs.msg.MoveRobotGoal(x, self.lastY, z, self.lastRoll, self.lastPitch, self.lastYaw,
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
