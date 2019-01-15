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
//PCL filtering less previous
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
//PCL transforming less previous
#include <pcl/common/eigen.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
//don less prvious
#include <string>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>
//euclidian less previous
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


//ROS_INTO 
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_RST "\033[0m"
#define BAR "----------------------------------------------------------------------------\n"
//vars
static int mode =1;//fix this later
static std::string nodeName("pass_through_filter");
static float xMinf, xMaxf, yMinf, yMaxf, zMinf, zMaxf;
static float alphaf  = -M_PI/2;
static float betaf = 0.0;
//static float gammaf = 0.0;
static float xTrans = 0.0;
static float yTrans = 0.0;
static float zTrans = 0.0;
static bool called = false;
static float boxMargin_setting=.2;
//outlier vars
static float statOutlier_meanK=75;
static float statOutlier_stdDev=.9;
//DoN vars
static double scale1=10;//The smallest scale to use in the DoN filter.
static double scale2=20;  //The largest scale to use in the DoN filter.
static double threshold=.1;  //The minimum DoN magnitude to threshold by
static double segradius=.5;//segment scene into clusters with given distance tolerance using euclidean clustering
static double minClusterSize=50;
static double maxClusterSize=80000;

static float leafSize=.01;
static float maxIterations=100;
static float distanceThreshold=.02;
static float clusterTollerance=.325;
//static float minClusterSize=100;
//static float maxClusterSize=200000;//default 40000 0 for test



//others clean vars TODO
static	int markerID=0;

//This node subscribes to a PointCloud2 topic, peforms a pass through filter, and republishes the point cloud. 
ros::Publisher pc2_pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

	// Create a container for the data and filtered data.
	/*
	   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	   pcl::PCLPointCloud2 cloud_filtered;
	   pcl_conversions::toPCL(*cloud_msg, *cloud);

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	called = true;

	pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
	 */
	ROS_INFO("PCP: leaf");	
	//NEW CONVERSION
	pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
	//euclidian cluster extraxion
	//these were moved here for scope of mode
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	//ddd pcl::PCDWriter writer;


	//----------------LEAF

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (temp_cloud);
	vg.setLeafSize (leafSize, leafSize, leafSize);//default (0.01f, 0.01f, 0.01f)//SETTING
	vg.filter (*cloud_filtered);

	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
	/*
	   sensor_msgs::PointCloud2 output;//create output container
	   pcl::PCLPointCloud2 temp_output;//create PCLPC2
	   pcl::toPCLPointCloud2(*cloud_filtered,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	   pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	   pc2_pub.publish (output);// Publish the data.
	 */
	//---------------passthrough
	/*
	   ROS_INFO("PCP: passThrough");
	   pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
	   pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
	   pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
	   pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
	 */

	temp_cloud=cloud_filtered;

	pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
	pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

	///pcl::PCLPointX cloud_filtered_x;
	//pcl::PCLPointCloud2 cloud_filtered_xz;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
	//crop x
	ptfilter.setInputCloud (temp_cloud);
	ptfilter.setFilterFieldName ("x");
	ptfilter.setFilterLimits (xMinf+boxMargin_setting, xMaxf-boxMargin_setting);
	ptfilter.filter (*cloud_filtered_x);
	//crop y
	ptfilter.setInputCloud(cloud_filtered_x);
	ptfilter.setFilterFieldName ("y");
	ptfilter.setFilterLimits (yMinf+boxMargin_setting, yMaxf-boxMargin_setting);
	ptfilter.filter (*cloud_filtered_xy);
	//crop z
	ptfilter.setInputCloud(cloud_filtered_xy);
	ptfilter.setFilterFieldName ("z");
	ptfilter.setFilterLimits (zMinf+boxMargin_setting, zMaxf-boxMargin_setting);//SETTING
	//ptfilter.setNegative (false);
	ptfilter.filter (*cloud_filtered_xyz);
	/*
	   sensor_msgs::PointCloud2 output;//create output container
	   pcl::PCLPointCloud2 temp_output;//create PCLPC2
	   pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	   pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	   pc2_pub.publish (output);// Publish the data.
	 */
	//------------------OUTLIER REMOVAL

	ROS_INFO("PCP: outlierRemoval");
	// Create a container for the data and filtered data.
	/*		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
			pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	//Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);
	 */
//	pcl::PCLPointCloud2 cloud_filtered_2;
//pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

//pcl::toPCLPointCloud2(*cloud_filtered_xyz,cloud);


pcl::PointCloud<pcl::PointXYZ> cloud_filtered_2;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ> ());
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_filtered_xyz);
	sor.setMeanK (statOutlier_meanK);//SETTING
	sor.setStddevMulThresh (statOutlier_stdDev);//SETTING
	sor.filter (cloud_filtered_2);

 sensor_msgs::PointCloud2 output;//create output container
	   pcl::PCLPointCloud2 temp_output;//create PCLPC2
	   pcl::toPCLPointCloud2(cloud_filtered_2,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	   pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
	   pc2_pub.publish (output);// Publish the data.


	
/*
	//convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCl(cloud_filtered_2,output);
	pcl_conversions::fromPCL(cloud_filtered_2,output);
	// Publish the data.
	pc2_pub.publish (output);
*/

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

	nh.getParam("settings/alpha", alphaf);
	nh.getParam("settings/beta", betaf);
	//	nh.getParam("settings/gamma", gammaf);
	nh.getParam("settings/xTrans", xTrans);
	nh.getParam("settings/yTrans", yTrans);
	nh.getParam("settings/zTrans", zTrans);


	nh.getParam("settings/don_scale1",scale1);
	nh.getParam("settings/don_scale2",scale2);
	nh.getParam("settings/don_threshold",threshold);
	nh.getParam("settings/don_segradius",segradius);

	nh.getParam("settings/don_minClusterSize",minClusterSize);
	nh.getParam("settings/don_maxClusterSize",maxClusterSize);


	nh.getParam("settings/euc_leafSize",leafSize);
	nh.getParam("settings/euc_maxIterations",maxIterations);
	nh.getParam("settings/euc_distancethreshold",distanceThreshold);
	nh.getParam("settings/euc_clusterTollerance",clusterTollerance);
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
/*
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
	if(myMode=="-1"||myMode=="leaf"||myMode=="Z"){
		mode=-1;
	}else if(myMode=="0"||myMode=="passThrough"||myMode=="A"){
		mode=0;
	}else if(myMode=="1"||myMode=="outlierRemoval"||myMode=="B"){
		mode=1;
	}else if(myMode=="2"||myMode=="transform"||myMode=="C"){
		mode=2;
	}else if(myMode=="3"||myMode=="don"||myMode=="D"){
		mode=3;
	}else if(myMode=="4"||myMode=="euc"||myMode=="E"){
		mode=4;
	}
*/
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe (sTopic.c_str(), 1, cloud_cb);

	ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic.c_str());
	// Create a ROS publisher for the output point cloud
	pc2_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic, 1);
	ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic.c_str());
	

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
}		

