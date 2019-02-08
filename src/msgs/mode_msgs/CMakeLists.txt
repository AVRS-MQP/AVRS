cmake_minimum_required(VERSION 2.8.3)
project(mode_msgs)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  message_generation)

add_service_files(
  DIRECTORY msg
  FILES
  Directory.srv
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
