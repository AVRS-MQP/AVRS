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
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <iostream>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/search.h>

//#include <Eigen>

#include <pcl/common/projection_matrix.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>

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
ros::Publisher cloud_pub2;//debug pub

int debugLevel =2;

void computePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	ROS_INFO("Doing a little math...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>());
	//	std::vector<pcl::Vertices> vertices;
	pcl::Vertices vt;

	//---Crop a cylinder out of the point cloud
	float x = 0;
	float y = 0;
	float z = 0;
	float rad = .1;//radius
	float dep =2;//depth

	//build cylinder TODO move to helper function
	//hullCloud->push_back(pcl::PointXYZ(x,y,z));//center
	//hullCloud->push_back(pcl::PointXYZ(x+rad,y,z));//right
	//hullCloud->push_back(pcl::PointXYZ(x-rad,y,z));//left
	//hullCloud->push_back(pcl::PointXYZ(x,y,z+rad));//top
	//hullCloud->push_back(pcl::PointXYZ(x,y,z-rad));//bottom
	//hullCloud->push_back(pcl::PointXYZ(x,y+dep,z));//center +depth
	hullCloud->push_back(pcl::PointXYZ(x+rad,y+dep,z));//right +depth
	hullCloud->push_back(pcl::PointXYZ(x-rad,y+dep,z));//left +depth
	hullCloud->push_back(pcl::PointXYZ(x,y+dep,z+rad));//top +depth
	hullCloud->push_back(pcl::PointXYZ(x,y+dep,z-rad));//bottom +depth

	hullCloud->push_back(pcl::PointXYZ(x+rad,y+.1,z));//right
	hullCloud->push_back(pcl::PointXYZ(x-rad,y+.1,z));//left
	hullCloud->push_back(pcl::PointXYZ(x,y+.1,z+rad));//top
	hullCloud->push_back(pcl::PointXYZ(x,y+.1,z-rad));//bottom




	//--Output debug cloud
	sensor_msgs::PointCloud2 output2;//create output container
	pcl::PCLPointCloud2 temp_output2;//create PCLPC2
	pcl::toPCLPointCloud2(*hullCloud,temp_output2);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output2,output2);//convert to ROS data type

	output2.header.frame_id="camera_depth_optical_frame";
	cloud_pub2.publish(output2);



	//vt stuff
/*	
	   vt.vertices.push_back(1);
	   vt.vertices.push_back(2);
	   vt.vertices.push_back(3);
	   vt.vertices.push_back(4);
	   vt.vertices.push_back(5);
	   vt.vertices.push_back(6);
	   vt.vertices.push_back(7);
	   vt.vertices.push_back(8);
	   vt.vertices.push_back(9);
	   vt.vertices.push_back(10);
	   vertices.push_back(vt);
*/
	 


	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(hullCloud);
	hull.setDimension(3);
	std::vector<pcl::Vertices> vertices;


	//pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>());


	hull.reconstruct(*hullCloud, vertices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cropResult(new pcl::PointCloud<pcl::PointXYZ>());
	//-perform actual filter
	pcl::CropHull<pcl::PointXYZ> cropHull;//create the filter object
	cropHull.setDim(3);//use a 3d hul
	cropHull.setInputCloud(cloud);//the cloud getting cropped
	cropHull.setHullIndices(vertices);//?
	cropHull.setHullCloud(hullCloud);//the crop shape
	cropHull.setCropOutside(true);//default is true, sets to remove outsid points
	//std::vector<int> indices;
	//cropHull.filter(indices);

	cropHull.filter(*cropResult);//compute and set output
	std::cout<< cropResult->size() <<std::endl;

	//--Output cropped cloud
	sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2
	pcl::toPCLPointCloud2(*cropResult,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type

	output.header.frame_id="camera_depth_optical_frame";
	cloud_pub.publish(output);



	//---GET Points for computing angles
	pcl::PointXYZ pMin,pMax;
	pcl::getMinMax3D (*cloud,pMin,pMax);

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
	ROS_INFO("%s: In cloud callback",nodeName.c_str());
	ROS_DEBUG("test");
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;// Create a container for the data and filtered data.
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	pcl_conversions::toPCL(*cloud_msg, *cloud);	//Convert to PCL data type

	//create PCLXYZ for

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ for input to be converted to 
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ

	computePose(temp_cloud2);
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

	//debug pub
	cloud_pub2=nh.advertise<sensor_msgs::PointCloud2> ("crop_zone",1);

	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic1.c_str());
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic2.c_str());
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();
}		

