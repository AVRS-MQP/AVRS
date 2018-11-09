#include <ros/ros.h>
#include <ros/master.h>

#include <stdio.h>
#include <iostream>

#include <follow_joint_trajectory_msgs/JointTrajectory.h>
#include <follow_joint_trajectory_msgs/JointTrajectoryPoint.h>


//publishers
ros::Publisher trajectory;

void sendTraj(const follow_joint_trajectory_msgs) {

follow_joint_trajectory_msgs:: JointTrajectory msg;
msg.headerstamp = ros::Time::now();

msg.joint_names = ["joint1"];
msg.points = [0 0 0];

}


int main (int argc, char** argv) { 

ros::init(argc, argv, "main");
ros::NodeHandle nh;

trajectory = nh.advertise<follow_joint_trajectory::goal>("goal",1); 


//ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, msg_cb);

rps::spin();

}

//Header header
//string[] joint_names
//trajectory_msgs/JointTrajectoryPoint desired
//trajectory_msgs/JointTrajectoryPoint actual
//trajectory_msgs/JointTrajectoryPoint error
