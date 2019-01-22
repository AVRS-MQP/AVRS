#include "ros/ros.h"
#include "directory_msgs/Directory.h"
static std::string nodeName("grey_scale_server");

bool greyScale(directory_msgs::Directory::Request  &req,
		directory_msgs::Directory::Response &res){
	//TODO delete res.Sum = req.A + req.B;
	ROS_INFO("request.input:[%s]", req.inputLoc.c_str());




	ROS_INFO("sending back response.output: [%s]", res.outputLoc.c_str());

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;
	nodeName=ros::this_node::getName();//Update Name

	ros::ServiceServer service = n.advertiseService("grey_scale", greyScale);
	ROS_INFO("Ready to process image.");
	ros::spin();

	return 0;
}
