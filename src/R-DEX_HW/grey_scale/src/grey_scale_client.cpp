#include "ros/ros.h"
#include "directory_msgs/Directory.h"
#include <cstdlib>
#include <ros/package.h>

int main(int argc, char **argv)
{


	ros::init(argc, argv, "grey_scale_client");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<directory_msgs::Directory>("grey_scale");
	directory_msgs::Directory srv;


	std::string path=ros::package::getPath("grey_scale");
	path.append("/images/input.png");

	if (argc > 2)
	{
		ROS_INFO("usage: grey_scale_client directory/path/file.png\n (.Or no argeuments to used default image)");
		return 1;
	}else if(argc > 1){
		srv.request.inputLoc = strdup(argv[1]);
	}else if (argc > 0){
		ROS_WARN("No input provided. Using default input: %s", path.c_str());
		srv.request.inputLoc=path.c_str();
	}

	if (client.call(srv))
	{
		ROS_INFO("Output: %s", srv.response.outputLoc.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service");
		return 1;
	}

	return 0;
}
