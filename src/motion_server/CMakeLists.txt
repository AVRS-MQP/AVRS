cmake_minimum_required(VERSION 2.8.3)
project(motion_server)
set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED 
COMPONENTS roscpp actionlib std_msgs actionlib_msgs motion_msgs
  roscpp
  tf
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface)

find_package(Boost REQUIRED COMPONENTS system)


include_directories(include ${catkin_INCLUDE_DIRS} )#${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}


#add_dependencies(
#  fibonacci_server_node
#  ${actionlib_tutorials_EXPORTED_TARGETS}
#)

#add_dependencies(fibonacci_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    motion_msgs	
)

add_executable(motion_server_node src/motion_server_node.cpp)

add_dependencies(motion_server_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  motion_server_node
    ${catkin_LIBRARIES}
)



