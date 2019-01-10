cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials_server)
set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED 
COMPONENTS roscpp actionlib std_msgs actionlib_msgs actionlib_tutorials_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} )#${Boost_INCLUDE_DIRS})





#add_dependencies(
#  fibonacci_server_node
#  ${actionlib_tutorials_EXPORTED_TARGETS}
#)

#add_dependencies(fibonacci_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib_tutorials_msgs	
)

add_executable(fibonacci_server_node src/fibonacci_server_node.cpp)

add_dependencies(fibonacci_server_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  fibonacci_server_node
    ${catkin_LIBRARIES}
)



