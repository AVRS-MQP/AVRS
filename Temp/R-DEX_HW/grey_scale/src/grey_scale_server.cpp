#include "ros/ros.h"
#include "directory_msgs/Directory.h"
#include <ros/package.h>
//#include "opencv2/core/version.hpp"// In ros Kinetic OpenCV3 is default. According to tutorial this include should allow coding in OpenCV3
//http://wiki.ros.org/opencv3
#include <stdio.h>

static std::string nodeName("grey_scale_server");
class GreyScale{
  private:
    bool greyScale(const char* input, const char* output){
      /*

	 cv::Mat image;//create holder for the image
	 try{
	 cv::image=imread(input,1);//read in the specified image file
	 }catch(const std::exception& e){
	 ROS_ERROR("Could not read file %s",input);
	 }
	 cv::Mat gray_image;//create holder for result
	 cv::cvtColor(image,gray_image, CV_BGR2GRAY);//apply grey-scale
	 cv::imwrite(output,gray_image);//write grey-scale image to specified file	


       */
    }

  public:
    bool imageCB(directory_msgs::Directory::Request  &req,
	directory_msgs::Directory::Response &res){//publically accessible class to handle service callback
      ROS_INFO("request.input:[%s]", req.inputLoc.c_str());
      //setup file path
      std::string path=ros::package::getPath("grey_scale");
      path.append("/images/output.png");

      greyScale(req.inputLoc.c_str(),path.c_str());//call helper function to do actuall processing
      res.outputLoc=path.c_str();//return the location the file will be saved to
      ROS_INFO("sending back response.output: [%s]", res.outputLoc.c_str());

      return true;
    }

};

int main(int argc, char **argv){
  //setup ROS node
  ros::init(argc, argv, nodeName);
  ros::NodeHandle n;
  nodeName=ros::this_node::getName();//Update Name

  GreyScale GS;//create object of class

  ros::ServiceServer service = n.advertiseService("grey_scale", &GreyScale::imageCB, &GS);//tie service to callback function inside a class
  ROS_INFO("Ready to process image.");
  ros::spin();

  return 0;
}
