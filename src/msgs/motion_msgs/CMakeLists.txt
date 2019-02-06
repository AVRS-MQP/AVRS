cmake_minimum_required(VERSION 2.8.3)
project(motion_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib geometry_msgs actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES 
  MoveRobot.action
  MoveRobotFast.action
  MoveRobotPose.action
  MoveRobotQuat.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

