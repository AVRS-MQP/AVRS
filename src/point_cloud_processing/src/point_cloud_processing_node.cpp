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
#include <lidar_utility_msgs/lidarUtilitySettings.h>
#include <lidar_utility_msgs/roadInfo.h>
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
//PCL filtering
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//PCL transforming
#include <pcl/common/eigen.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>

#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
static int mode =1;//fix this later
static std::string nodeName("pass_through_filter");
static 	float xMinf, xMaxf, yMinf, yMaxf, zMinf, zMaxf;
static bool called = false;
static float boxMargin_setting=.2;

static float statOutlier_meanK=75;
static float statOutlier_stdDev=.9;
//This node subscribes to a PointCloud2 topic, peforms a pass through filter, and republishes the point cloud. 
ros::Publisher pc2_pub;

int debugLevel =2;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	//pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;

	//Callback for filtering and republishing recived data

	ROS_DEBUG_COND(debugLevel==2,"%s: In Callback",nodeName.c_str());
	ROS_INFO("mode: %d",mode);
	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	called = true;


	// Do data processing here...

	//Convert to PCL data type
	//pcl_conversions::toPCL(*cloud_msg, *cloud);

	//?

	


pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
		

	if(mode==0||mode==4){//passThrough or all
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ


		pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
		pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

		///pcl::PCLPointX cloud_filtered_x;
		//pcl::PCLPointCloud2 cloud_filtered_xz;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices

		ptfilter.setInputCloud (temp_cloud);
		ptfilter.setFilterFieldName ("x");
		ptfilter.setFilterLimits (xMinf+boxMargin_setting, xMaxf-boxMargin_setting);
		ptfilter.filter (*cloud_filtered_x);

		ptfilter.setInputCloud(cloud_filtered_x);
		ptfilter.setFilterFieldName ("y");
		ptfilter.setFilterLimits (yMinf+boxMargin_setting, yMaxf-boxMargin_setting);
		ptfilter.filter (*cloud_filtered_xy);

		ptfilter.setInputCloud(cloud_filtered_xy);
		ptfilter.setFilterFieldName ("z");
		ptfilter.setFilterLimits (zMinf+boxMargin_setting, zMaxf-boxMargin_setting);//SETTING
		//ptfilter.setNegative (false);
		ptfilter.filter (*cloud_filtered_xyz);
ROS_INFO("VALUE: %f",yMaxf);

sensor_msgs::PointCloud2 output;//create output container
		pcl::PCLPointCloud2 temp_output;//create PCLPC2
		pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
		pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
		pc2_pub.publish (output);// Publish the data.

	
	}
	if(mode==1||mode==4){//outlerRemoval or all

	// Create a container for the data and filtered data.
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


	// Do data processing here...

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	pcl::PCLPointCloud2 cloud_filtered;
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setMeanK (statOutlier_meanK);//SETTING
		sor.setStddevMulThresh (statOutlier_stdDev);//SETTING
		sor.filter (cloud_filtered);
//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered,output);
	pcl_conversions::fromPCL(cloud_filtered,output);
	// Publish the data.
	pc2_pub.publish (output);

	}
	if (mode==2||mode==4){//transform or all

		ROS_INFO("beta");
		// Create a container for the data.
		sensor_msgs::PointCloud2 output2;
		
		float theta =-M_PI/2;

		pcl::PointCloud<pcl::PointNormal> temp_cloud2;
		/*
		   float theta =M_PI/2;
		   Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		   transform(0,0)= cos(theta);
		   transform(0,1)= -sin(theta);
		   transform(1,0)= sin(theta);
		   transform(1,1)= cos(theta);
		 */
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ> ());




		//		printf ("Transform: Matrix4f\n");
		//		std::cout << transform << std::endl;



		//method 2

		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.translation() << 0.0,0.0,0.0;//2.5 meters x
		transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));//rotate theata aboutz

		// Print the transformation
		printf ("\nMethod #2: using an Affine3f\n");
		std::cout << transform_2.matrix() << std::endl;

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::transformPointCloudWithNormals(*temp_cloud,*cloud_transformed,transform_2);

		if(mode==2){
			pcl::PCLPointCloud2 temp_output;//create PCLPC2
			pcl::toPCLPointCloud2(*cloud_transformed,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
			pcl_conversions::fromPCL(temp_output,output2);//convert to ROS data typeos::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, message_cb);
			// Publish the data.
			pc2_pub.publish (output2);
}

}
if (mode==3||mode==4){//don or all

}
if(mode==4){

}
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

	//update settings
	nh.getParam("settings/passThrough_Xmin", xMinf);
	nh.getParam("settings/passThrough_Xmax", xMaxf);
	nh.getParam("settings/passThrough_Ymin", yMinf);
	nh.getParam("settings/passThrough_Ymax", yMaxf);
	nh.getParam("settings/passThrough_Zmin", zMinf);
	nh.getParam("settings/passThrough_Zmax", zMaxf);

	nh.getParam("settings/statOutlier_meanK", statOutlier_meanK);
	nh.getParam("settings/statOutlier_stdDev", statOutlier_stdDev);

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
	//if(mode==3){
	//ros::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, message_cb);
	//}
	//ros::Subscriber sub3 = nh.subscribe("lidar_utility_settings", 1, settings_cb);

	ros::spin();
}		

