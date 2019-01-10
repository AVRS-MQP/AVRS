cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials_client)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED 
COMPONENTS roscpp actionlib actionlib_msgs actionlib_tutorials_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})





#add_dependencies(
#  fibonacci_client_node
#  ${actionlib_tutorials_client_EXPORTED_TARGETS}
#)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib_tutorials_msgs	
)

add_executable(fibonacci_client_node src/fibonacci_client_node.cpp)

add_dependencies(fibonacci_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( 
  fibonacci_client_node
  ${catkin_LIBRARIES}
)

