
#include <ros/ros.h>
//#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>

#include <math.h>
#include <tf/transform_datatypes.h>


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

	//std::string base_frame;
	//private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

	//ScanNPlan app(nh);

	//ros::Duration(.5).sleep();  // wait for the class to initialize
	//app.start(base_frame);

	//create quaternion

	tf::Quaternion q_rot;

	float r=0;
	float p=180;
	float y=0;


	r=r*(M_PI/180);
	p=p*(M_PI/180);
	y=y*(M_PI/180);

	q_rot = tf::createQuaternionFromRPY(r, p, y);//roll(x), pitch(y), yaw(z),

	ROS_WARN("A");
	geometry_msgs::Pose poseEOAT;

	quaternionTFToMsg(q_rot,poseEOAT.orientation);
	//poseEOAT.orientation=q_rot;
	poseEOAT.position.x=-.75;
	poseEOAT.position.y=-.75;
	poseEOAT.position.z=1;

	std::string base_frame = "/base_link";

	geometry_msgs::Pose move_target = poseEOAT;
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	// Plan for robot to move to part
	move_group.setPoseReferenceFrame(base_frame);
	move_group.setPoseTarget(move_target);
	ROS_WARN("b.4");
	//move_group.();	
	move_group.move();
	ROS_WARN("b.5");
	ros::waitForShutdown();
}
