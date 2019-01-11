cmake_minimum_required(VERSION 2.8.3)
project(motion_planning_client)

find_package(catkin REQUIRED 
COMPONENTS roscpp actionlib actionlib_msgs motion_msgs

  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  tf
  force_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#catkin_install_python(PROGRAMS scripts/main_node DESTINATION ${bin})

#include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    rospy
    message_runtime
    motion_msgs	
    force_msgs
)
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/main_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries( 
#  main_node
#  ${catkin_LIBRARIES}
#)

