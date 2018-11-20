/* AVRS-MQP: motion_action_server -g
 *  Maintainer: avrs.mqp@gmail.com
 *  Authors: Nikolas Gamarra, Ryan O'Brien
 */

#include <ros/ros.h>
//#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <math.h>
#include <tf/transform_datatypes.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//action server inculdes

#include <actionlib/server/simple_action_server.h>
#include <chores/DoDishesAction.h>  // Note: "Action" is appendedss
#include <chores/MoveRobotAction.h>

//Location variables set at launch
float roll, pitch, yaw;
float x, y, z;

//action server
typedef actionlib::SimpleActionServer<chores::MoveRobotAction> Server;


class MoveAction{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<chores::MoveRobotAction> as_;

		std::string action_name_;
		chores::MoveRobotAction feedback_;
		chores::MoveRobotAction result_;

	public:
		MoveAction(std::string name) :
			as_(nh_,name, boost::bind(&MoveAction::executeCB, this, _1),false),action_name_(name)
	{
		as_.start();
	}
		~MoveAction(void)
		{
		}


		void executeCB(const chores::MoveRobotAction &goal){
			//create quaternion
			tf::Quaternion q_rot;
			tf::TransformListener listener;

			//pull all the values from goal
			roll=goal->roll;//TODO
			pitch=goal.pitch;
			yaw=goal.yaw;
			x=goal.x;
			y=goal.y;
			z=goal.z;
			/*
			   nh.getParam("move_pose/roll",roll);
			   nh.getParam("move_pose/pitch",pitch);
			   nh.getParam("move_pose/yaw",yaw);

			   nh.getParam("move_pose/x_pos",x);
			   nh.getParam("move_pose/y_pos",y);
			   nh.getParam("move_pose/z_pos",z);
			 */

			//convert deg to rad
			roll=roll*(M_PI/180);
			pitch=pitch*(M_PI/180);
			yaw=yaw*(M_PI/180);

			//create and fill pose	
			q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);//roll(x), pitch(y), yaw(z),
			geometry_msgs::Pose poseEOAT;
			quaternionTFToMsg(q_rot,poseEOAT.orientation);
			poseEOAT.position.x= x;
			poseEOAT.position.y= y;
			poseEOAT.position.z= z;

			//transform between joint 6 and base frame
			/*
			   tf::Transform tool_trans;
			   tf::poseMsgToTF(poseEOAT, tool_trans);

			   tf::StampedTransform j6_trans;
			   listener.waitForTransform("/base_link","/link_6",ros::Time(0), ros::Duration(4.0));

			   listener.lookupTransform("/base_link", "/link_6", ros::Time(0), j6_trans);

			   tf::Transform j6_to_base;
			   j6_to_base = j6_trans * tool_trans;

			   tf::poseTFToMsg(j6_to_base, poseEOAT);
			 */

			std::string base_frame = "/base_link";

			geometry_msgs::Pose move_target = poseEOAT;
			moveit::planning_interface::MoveGroupInterface move_group("manipulator");
			// Plan for robot to move to part
			move_group.setPoseReferenceFrame(base_frame);
			move_group.setPoseTarget(move_target);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			move_group.move();

			//as->setSucceeded();
		}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_action_server");
	MoveAction move("move");
	ros::spin();

	return 0;
}
