cmake_minimum_required(VERSION 2.8.3)
project(motion_action_server)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs)
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

add_executable(motion_server src/motion_server.cpp)

target_link_libraries(
  motion_server
    ${catkin_LIBRARIES}
)

add_dependencies(
  motion_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)



