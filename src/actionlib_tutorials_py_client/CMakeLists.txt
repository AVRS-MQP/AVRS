cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials_py_client)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED 
COMPONENTS roscpp actionlib actionlib_msgs actionlib_tutorials_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_python_setup()

#add_dependencies(
#  fibonacci_client_node
#  ${actionlib_tutorials_client_EXPORTED_TARGETS}
#)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib_tutorials_msgs	
)

add_dependencies( ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(fibonacci_client_node src/fibonacci_client_node.cpp)

#add_dependencies(fibonacci_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/fib_test
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#target_link_libraries( 
#  main_node
#  ${catkin_LIBRARIES}
#)

