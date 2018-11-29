cmake_minimum_required(VERSION 2.8.3)
project(motion_action_server_msgs)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs   

)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES MoveRobot.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


