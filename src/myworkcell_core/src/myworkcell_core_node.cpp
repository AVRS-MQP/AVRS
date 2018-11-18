
#include <ros/ros.h>
//#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include <math.h>
#include <tf/transform_datatypes.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

//Location variables set at launch
float roll, pitch, yaw;
float x, y, z;


/*
   class ScanNPlan
   {
   public:
   ScanNPlan(ros::NodeHandle& nh)
   {
   vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
   }

   void start(const std::string& base_frame)
   {
   ROS_INFO("Attempting to localize part");
// Localize the part
myworkcell_core::LocalizePart srv;
srv.request.base_frame = base_frame;
ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

if (!vision_client_.call(srv))
{
ROS_ERROR("Could not localize part");
return;
}
ROS_INFO_STREAM("part localized: " << srv.response);

geometry_msgs::Pose move_target = srv.response.pose;
moveit::planning_interface::MoveGroupInterface move_group("manipulator");

// Plan for robot to move to part
move_group.setPoseReferenceFrame(base_frame);
move_group.setPoseTarget(move_target);
move_group.move();
}

private:
// Planning components
ros::ServiceClient vision_client_;
};
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "myworkcell_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_node_handle("~");
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO("workcell_core init");

	//tf::Transform j6_to_base;

	//std::string base_frame;
	//private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

	//ScanNPlan app(nh);

	//ros::Duration(.5).sleep();  // wait for the class to initialize
	//app.start(base_frame);

	//create quaternion

	tf::Quaternion q_rot;
	tf::TransformListener listener;

	nh.getParam("move_pose/roll",roll);
	nh.getParam("move_pose/pitch",pitch);
	nh.getParam("move_pose/yaw",yaw);

	nh.getParam("move_pose/x_pos",x);
	nh.getParam("move_pose/y_pos",y);
	nh.getParam("move_pose/z_pos",z);

	roll=roll*(M_PI/180);
	pitch=pitch*(M_PI/180);
	yaw=yaw*(M_PI/180);

	q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);//roll(x), pitch(y), yaw(z),
	geometry_msgs::Pose poseEOAT;

	quaternionTFToMsg(q_rot,poseEOAT.orientation);
	//poseEOAT.orientation=q_rot;
	poseEOAT.position.x= x;
	poseEOAT.position.y= y;
	poseEOAT.position.z= z;

	//transform between joint 6 and base frame
	tf::Transform tool_trans;
	tf::poseMsgToTF(poseEOAT, tool_trans);

	tf::StampedTransform j6_trans;
        listener.waitForTransform("/base_link","/link_6",ros::Time(0), ros::Duration(4.0));

	listener.lookupTransform("/base_link", "/link_6", ros::Time(0), j6_trans);

	tf::Transform j6_to_base;
	j6_to_base = j6_trans * tool_trans;

	tf::poseTFToMsg(j6_to_base, poseEOAT);


	std::string base_frame = "/base_link";

 	// moveit_visual_tools::MoveItVisualTools visual_tools("abb_link0");

	//visual_tools.publishAxisLabeled(poseEOAT, "pose1")
	//visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("manipulator");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();



	geometry_msgs::Pose move_target = poseEOAT;
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	// Plan for robot to move to part
	move_group.setPoseReferenceFrame(base_frame);
	move_group.setPoseTarget(move_target);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	//move_group.();	
	//move_group.move();
	
	ros::waitForShutdown();
}
