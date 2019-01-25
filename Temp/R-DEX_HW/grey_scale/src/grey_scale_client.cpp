#include "ros/ros.h"
#include "directory_msgs/Directory.h"
#include <cstdlib>
#include <ros/package.h>

int main(int argc, char **argv)
{
  //setup ROS node and service client
  ros::init(argc, argv, "grey_scale_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<directory_msgs::Directory>("grey_scale");
  //prepair default path
  directory_msgs::Directory srv;
  std::string path=ros::package::getPath("grey_scale");
  path.append("/images/input.png");

  if (argc > 2){//check user inputs
    ROS_INFO("usage: grey_scale_client directory/path/file.png\n (.Or no argeuments to used default image)");
    return 1;
  }else if(argc > 1){//user provided file
    srv.request.inputLoc = strdup(argv[1]);
  }else if (argc > 0){//no arguments use default file
    ROS_WARN("No input provided. Using default input: %s", path.c_str());
    srv.request.inputLoc=path.c_str();
  }

  if (client.call(srv)){//call and print the response of the server
    ROS_INFO("Output: %s", srv.response.outputLoc.c_str());
  }else{//throw error if server isnt up
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
