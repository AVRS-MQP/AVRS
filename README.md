#AVRS-MQP
Automatic Vehicle Recharging Station (AVRS)

In the future when levle 5 autonomous vehicles exist, solutions will be needed to automate charging in order to create systems completely independent from human operation. 

This Major Qualifying Project (MQP) aims to design, manufacure, and program such a system capable of opening the charging port flap and charging vehicle of different charging port types. 

Advisors: Craig Putnam, Jie Fu
Students: Matthew Fortmeyer, Nikolas Gamarra, Ryan O’Brien, Jacob Remz


## ROS packages/nodes and launch files

- ROS-I
Launch the ROS-I stack to connect to the robot and visualize in rviz:

```
roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.100.100
```

- motion_server
This node recives action messages from the action client and sends motion requests to the ROS-I stack using move_group
```
roslaunch motion_server motion_server.launch 
```

- motion_planning_client
This node is the main controll node written in python.  When in normal operation it will read an image and draw it on the white board. In debug mode it goes to manually  prgorammed locations.

```
roslaunch motion_planning_client motion_client.launch 
```

- point_cloud_processing
Masages point cloud data (leafs,crops,removes outliers). 

```
roslaunch point_cloud_processing point_cloud_processing.launch 
```

- point_cloud_processing_experimental
A playground for rapidly prototpying new point cloud filtering and interpretation.

```
roslaunch point_cloud_processing_experimental point_cloudv3.launch 
```

- master_launcher
Package with various micilanous launch files and launch files that launch others

```
TBD/varies
```

- msgs
Automatically generated custom msgs

# NOTES
```
to visualize the robot run

roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.100.100

roslaunch moveit_setup_assistant setup_assistant.launch

roslaunch master_launcher simple_kinect.launch 

```




## Depends on

ROS-I
PCL
Open CV
MoveIt
TRAC-IK Kinematics Solver
http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html


To clone with all dependend submodules
git clone URL
cd FOLDER
git submodule init
git submodule update





