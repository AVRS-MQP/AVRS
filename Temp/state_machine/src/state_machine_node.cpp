#include <ros/ros.h>
#include <ros/master.h>

#include <stdio.h>
#include <iostream>

#include <geometry_msgs/Pose.h>

//publishers
ros::Publisher trajectory;


int main (int argc, char** argv) { 

ros::init(argc, argv, "main");
ros::NodeHandle nh;


static const std::string PLANNING_GROUP = "irb1600";
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//create a target
move_group.setStartState(*move_group.getCurrentState());
geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;

move_group.setPoseTarget(target_pose1);


//
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


//actually move the robot
//move_group.move();

//ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, msg_cb);

rps::spin();

}

//Header header
//string[] joint_names
//trajectory_msgs/JointTrajectoryPoint desired
//trajectory_msgs/JointTrajectoryPoint actual
//trajectory_msgs/JointTrajectoryPoint error
