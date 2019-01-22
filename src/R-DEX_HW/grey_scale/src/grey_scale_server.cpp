#include "ros/ros.h"
#include "directory_msgs/Directory.h"
#include <ros/package.h>
#include "opencv2/core/version.hpp"// In ros Kinetic OpenCV3 is default. According to tutorial this include should allow coding in OpenCV3
//http://wiki.ros.org/opencv3

static std::string nodeName("grey_scale_server");
class GreyScale{
	private:
	bool greyScale(const char* input, const char* output){
	
	}

	public:
	bool imageCB(directory_msgs::Directory::Request  &req,
			directory_msgs::Directory::Response &res){
		//TODO delete res.Sum = req.A + req.B;
		ROS_INFO("request.input:[%s]", req.inputLoc.c_str());

		std::string path=ros::package::getPath("grey_scale");
		path.append("/images/output.png");
		
		greyScale(req.inputLoc.c_str(),path.c_str());
		res.outputLoc=path.c_str();
		ROS_INFO("sending back response.output: [%s]", res.outputLoc.c_str());

		return true;
	}

};

int main(int argc, char **argv){
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;
	nodeName=ros::this_node::getName();//Update Name

	GreyScale GS;

	ros::ServiceServer service = n.advertiseService("grey_scale", &GreyScale::imageCB, &GS);
	ROS_INFO("Ready to process image.");
	ros::spin();

	return 0;
}
