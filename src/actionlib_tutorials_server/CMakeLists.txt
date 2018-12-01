cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials_server)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs actionlib_tutorials_msgs)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fibonacci_server_node src/fibonacci_server_node.cpp)

target_link_libraries(
  fibonacci_server_node
    ${catkin_LIBRARIES}
)

#add_dependencies(
#  fibonacci_server_node
#  ${actionlib_tutorials_EXPORTED_TARGETS}
#)

add_dependencies(fibonacci_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package()

