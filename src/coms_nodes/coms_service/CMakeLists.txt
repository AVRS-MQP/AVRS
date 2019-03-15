cmake_minimum_required(VERSION 2.8.3)
project(comms_service)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  coms_msgs
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   coms_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coms
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node src/coms_service_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

