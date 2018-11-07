/*Point Cloud Processing
 *Nikolas Xarles Gamarra
 */ 
//ROS
#include <ros/ros.h>
#include <ros/master.h>
//C
#include <stdio.h>
#include <iostream>
//custom msgs
//PCL specific includes
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>
#include <pcl/visualization/pcl_visualizer.h>


//ROS_INTO 
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
//vars
static int mode =1;//fix this later
static std::string nodeName("temp_name");
static float xMinf, xMaxf, yMinf, yMaxf, zMinf, zMaxf;

//publishers
ros::Publisher pc2_pub;

int debugLevel =2;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
}
  int
main (int argc, char** argv)
{
  //initialize default topics for subscribing and publishing
  const std::string defaultCloudSubscriber("cloud_pcd");
  const std::string defaultMsgSubscriber("plane_segmented_msg");
  const std::string defaultCloudPublisher("passThrough_filtered");
  const std::string defaultMode("1");

  // Initialize ROS
  ros::init (argc, argv, nodeName);
  ros::NodeHandle nh;

  nodeName = ros::this_node::getName();//Update name


  //set parameters on new name
  const std::string subscriberParamName(nodeName + "/subscriber");
  const std::string subscriberParamName2(nodeName + "/msgSubscriber");
  const std::string publisherParamName(nodeName + "/publisher");
  const std::string modeParamName(nodeName + "/mode");
  printf(COLOR_BLUE BAR COLOR_RST);
  ROS_INFO("Node Name: %s",nodeName.c_str());

  //Create variables that control the topic names
  std::string sTopic;
  std::string pTopic;
  std::string myMode;
  std::string sTopic2;

  if(nh.hasParam(subscriberParamName)){//Check if the user specified a subscription topic
    nh.getParam(subscriberParamName,sTopic);
    printf(COLOR_GREEN BAR COLOR_RST);
    ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
  }else{
    sTopic=defaultCloudSubscriber;//set to default if not specified
    printf(COLOR_RED BAR COLOR_RST);
    ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),subscriberParamName.c_str(), sTopic.c_str());
  }

  if(nh.hasParam(subscriberParamName2)){//Check if the user specified a subscription topic for msgs
    nh.getParam(subscriberParamName2,sTopic2);
    printf(COLOR_GREEN BAR COLOR_RST);
    ROS_INFO("%s: A param has been set **%s** \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
  }else{
    sTopic2=defaultMsgSubscriber;//set to default if not specified
    printf(COLOR_RED BAR COLOR_RST);
    ROS_INFO("%s: No param set **%s**  \nSetting subsceiber2 to: %s",nodeName.c_str(),subscriberParamName2.c_str(), sTopic2.c_str());
  }

  if(nh.hasParam(publisherParamName)){//Check if the user specified a publishing topic
    printf(COLOR_GREEN BAR COLOR_RST);
    nh.getParam(publisherParamName,pTopic);
    ROS_INFO("%s: A param has been set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
  }else{printf(COLOR_RED BAR COLOR_RST);
    pTopic=defaultCloudPublisher;//set to default if not specified
    ROS_INFO("%s: No param set **%s** \nSetting publisher to: %s",nodeName.c_str(),publisherParamName.c_str(), pTopic.c_str());
  }

  if(nh.hasParam(modeParamName)){//Check if the user specified a subscription topic
    nh.getParam(modeParamName,myMode);
    printf(COLOR_GREEN BAR COLOR_RST);
    ROS_INFO("%s: A param has been set **%s** \nSetting mode to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
  }else{
    sTopic=defaultMode;//set to default if not specified
    printf(COLOR_RED BAR COLOR_RST);
    ROS_INFO("%s: No param set **%s**  \nSetting subsceiber to: %s",nodeName.c_str(),modeParamName.c_str(), myMode.c_str());
  }

  ROS_INFO("modex: %s",myMode.c_str());

  //Clears the assigned parameter. Without this default will never be used but instead the last spefified topic
  nh.deleteParam(subscriberParamName);
  nh.deleteParam(publisherParamName);
  nh.deleteParam(modeParamName);
  nh.deleteParam(subscriberParamName2);
  if(myMode=="0"||myMode=="passThrough"||myMode=="A"){
    mode=0;
  }else if(myMode=="1"||myMode=="outlierRemoval"||myMode=="B"){
    mode=1;
  }else if(myMode=="2"||myMode=="transform"||myMode=="C"){
    mode=2;
  }else if(myMode=="3"||myMode=="don"||myMode=="D"){
    mode=3;
  }else if(myMode=="4"||myMode=="all"||myMode=="E"){
    mode=4;
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

  ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
  // Create a ROS publisher for the output point cloud
  pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
  ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());

  ros::spin();
}		

