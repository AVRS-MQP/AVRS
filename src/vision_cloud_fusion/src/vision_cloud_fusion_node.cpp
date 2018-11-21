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

#include <geometry_msgs/Pose.h>

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
ros::Publisher pose_pub;
ros::Publisher cloud_pub;

int debugLevel =2;

void computePose(){



	pcl::PointXYZ pMin,pMax;
	pcl::getMinMax3D (*cloud_p,pMin,pMax);

	/*
	   lidar_utility_msgs::roadInfo msg;

	   msg.headerstamp = ros::Time::now();
	   msg.header.frame_id = "/world";
	   msg.xMax=pMax.x;
	   msg.xMin=pMin.x;
	   msg.yMax=pMax.y;
	   msg.yMin=pMin.y;
	   msg.zMax=pMax.z;
	   msg.zMin=pMin.z;



	   sensor_msgs::PointCloud2 output;//create output container
	   pcl::PCLPointCloud2 temp_output;//create PCLPC2
	   pcl::toPCLPointCloud2(*cloud_p,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	   pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	   pc2_pub.publish (output);// Publish the data.


	 */
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	ROS_INFO("%s: In Callback",nodeName.c_str());

	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;// Create a container for the data and filtered data.
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	pcl_conversions::toPCL(*cloud_msg, *cloud);	//Convert to PCL data type

	//create PCLXYZ for
	i
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ for input to be converted to 
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
}

void vision_cb(const geometry_msgs::Pose& pose_msg){

}
	int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, nodeName);
	ros::NodeHandle nh;

	nodeName = ros::this_node::getName();//Update name

	//set parameters on new name
	const std::string subscriberParamName1(nodeName + "/cloudSub");
	const std::string subscriberParamName2(nodeName + "/visionSub");
	const std::string publisherParamName1(nodeName + "/posePub");
	const std::string publisherParamName2(nodeName + "/cloudPub");

	printf(COLOR_BLUE BAR COLOR_RST);
	ROS_INFO("Node Name: %s",nodeName.c_str());

	//Create variables that control the topic names
	std::string sTopic1;
	std::string sTopic2;
	std::string pTopic1;
	std::string pTopic2;

	//pull the node specific params	
	nh.getParam(subscriberParamName1,sTopic1);
	nh.getParam(subscriberParamName2,sTopic2);
	nh.getParam(publisherParamName1,pTopic1);
	nh.getParam(publisherParamName2,pTopic2);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub1 = nh.subscribe (sTopic1.c_str(), 1, cloud_cb);
	ros::Subscriber sub2 = nh.subscribe (sTopic2.c_str(), 1, vision_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic1.c_str());
	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic2.c_str());

	// Create a ROS publisher for the output point cloud
	pose_pub = nh.advertise<geometry_msgs::Pose> (pTopic1, 1);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic2, 1);

	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic1.c_str());
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic2.c_str());
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
}		

