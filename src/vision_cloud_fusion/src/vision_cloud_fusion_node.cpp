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
#include <pcl/filters/statistical_outlier_removal.h>


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

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/common/centroid.h>


#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <stdio.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
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

//outlier vars
static float statOutlier_meanK=75;
static float statOutlier_stdDev=.9;


//publishers
ros::Publisher pose_pub;
ros::Publisher cloud_pub;
ros::Publisher cloud_pub2;//debug pub
ros::Publisher cloud_pub3;//multi debug pubs

static int debugLevel =2;
class Fusion{
  private:
    void computePose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
      if(debugLevel >1){
	ROS_INFO("Size at start of computePose:");
	std::cout<< cloud->size() <<std::endl;
	ROS_INFO("Doing a little math...");
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>());

      //	std::vector<pcl::Vertices> vertices;
      pcl::Vertices vt;

      //---Crop a cylinder out of the point cloud
      float x = -0.25;
      float y = -0.06;
      float z = 0; 
      float rad = .175;//radius//.1
      float dep =2;//depth

      //build cylinder TODO move to helper function
      hullCloud->push_back(pcl::PointXYZ(x,y,z));//center
      hullCloud->push_back(pcl::PointXYZ(x+rad,y,z));//right
      hullCloud->push_back(pcl::PointXYZ(x-rad,y,z));//left
      hullCloud->push_back(pcl::PointXYZ(x,y+rad,z));//top
      hullCloud->push_back(pcl::PointXYZ(x,y-rad,z));//bottom
      hullCloud->push_back(pcl::PointXYZ(x,y,z+dep));//center +depth
      hullCloud->push_back(pcl::PointXYZ(x+rad,y,z+dep));//right +depth
      hullCloud->push_back(pcl::PointXYZ(x-rad,y,z+dep));//left +depth
      hullCloud->push_back(pcl::PointXYZ(x,y+rad,z+dep));//top +depth
      hullCloud->push_back(pcl::PointXYZ(x,y-rad,z+dep));//bottom +depth

      //--Output debug cloud
      sensor_msgs::PointCloud2 output2;//create output container
      pcl::PCLPointCloud2 temp_output2;//create PCLPC2
      pcl::toPCLPointCloud2(*hullCloud,temp_output2);//convert from PCLXYZ to PCLPC2 must be pointer input
      pcl_conversions::fromPCL(temp_output2,output2);//convert to ROS data type

      output2.header.frame_id="camera_depth_optical_frame";
      cloud_pub2.publish(output2);

      //---Do Hull Crop
      pcl::ConvexHull<pcl::PointXYZ> hull;
      hull.setInputCloud(hullCloud);
      hull.setDimension(3);
      std::vector<pcl::Vertices> vertices;

      hull.reconstruct(*hullCloud, vertices);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cropResult(new pcl::PointCloud<pcl::PointXYZ>());
      //--perform actual filter
      pcl::CropHull<pcl::PointXYZ> cropHull;//create the filter object
      cropHull.setDim(3);//use a 3d hul
      cropHull.setInputCloud(cloud);//the cloud getting cropped
      cropHull.setHullIndices(vertices);//?
      cropHull.setHullCloud(hullCloud);//the crop shape
      cropHull.setCropOutside(true);//default is true, sets to remove outsid points

      cropHull.filter(*cropResult);//compute and set output
      if(debugLevel>1){ROS_INFO("size cropResult");}
      std::cout<< cropResult->size() <<std::endl;

      if(cropResult->size()!=0){

	//remove outliers
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cropResult);
ROS_INFO("MEANK%f",statOutlier_meanK);
	sor.setMeanK (statOutlier_meanK);//SETTING
	sor.setStddevMulThresh (statOutlier_stdDev);//SETTING
	sor.filter (cloud_filtered);

	//--Output cropped cloud
	sensor_msgs::PointCloud2 output;//create output container
	pcl::PCLPointCloud2 temp_output;//create PCLPC2
	pcl::toPCLPointCloud2(cloud_filtered,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
	pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type

	output.header.frame_id="camera_depth_optical_frame";
	cloud_pub.publish(output);

	//---calculate normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);
	if(debugLevel>1){
	  std::cout<<"Size of cloud_normals:"<< cloud_normals->size()<<std::endl;
	}
	//---GET Points for computing angles
	pcl::PointXYZ pMin,pMax;
	pcl::getMinMax3D (cloud_filtered,pMin,pMax);

	//---plane seg
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);


	//::PointCloud<pcl::PointXYZ> linaccpc;//TODO make it so seg actually uses outlers removed
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_PTR (&cloud_filtered);

	seg.setInputCloud (cropResult);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	  ROS_ERROR ("Could not estimate a planar model for the given dataset.");
	  //return (-1);
	}
	if(debugLevel>2){
	  // (in ax + by + cz + d = 0 form).
	  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
	    << coefficients->values[1] << " "
	    << coefficients->values[2] << " " 
	    << coefficients->values[3] << std::endl;
	}
	// vars for pose
	float roll=0;
	float pitch=0;
	float yaw=0;
	//calc angles rpy from slopes xyz
	// note abc not associated with angles of thier axis


	//WAS: 1, 2 ,0
	roll=atan2(coefficients->values[0],1);
	pitch=atan2(coefficients->values[1],1);
	yaw=atan2(coefficients->values[2],1);//seems correct

	//print all the inliers
	if(debugLevel>=3){
	  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	  for (size_t i = 0; i < inliers->indices.size (); ++i)
	    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
	      << cloud->points[inliers->indices[i]].y << " "
	      << cloud->points[inliers->indices[i]].z << std::endl;
	}


	// TODO FIX to centroid or mean of all z vals or remove outliers need to fix values over edge errors
	//set XYZ

	//the current dept dist
	y=pMin.y+((pMax.y-pMin.y)/2);

	//other values tbd if set here
	x=pMin.x+((pMax.x-pMin.x)/2);
	z=pMin.z+((pMax.z-pMin.z)/2);


	//mess with orientation


	//yaw=0;
	/*
	//constant modifiers to angle
	float change = (M_PI/180)*3;
	roll=roll+change;

	change = (M_PI/180)*5;
	pitch=pitch-change;
	yaw=yaw-change;
	//roll=roll*-1;
	//roll=0;
	pitch=0;
	//pitch=0;
	//yaw=yaw*-1;

	 */
	//---Build Pose/transform
	//convert deg to rad
	/*	roll=roll*(M_PI/180);
		pitch=pitch*(M_PI/180);
		yaw=yaw*(M_PI/180);
	 */	

	tf::Quaternion q_rot;
	q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);//roll(x), pitch(y), yaw(z),

//attempt to clear out roll field

   q_rot.normalize();


	geometry_msgs::Pose poseFlap;
	//poseFlap.header.frame_id="camera_depth_optical_frame";

	quaternionTFToMsg(q_rot,poseFlap.orientation);
	//poseFlap.pose.orientation=q_rot;
	poseFlap.position.x= x;
	poseFlap.position.y= y;
	poseFlap.position.z= z;
	//---Publish i
	//pose_pub.publish(poseFlap);
	static tf::TransformBroadcaster br;
	tf::Transform transf;
	tf::Quaternion q;
	q.setRPY(roll,pitch,yaw);

	transf.setOrigin(tf::Vector3(x,y,z));
	transf.setRotation(q);
	br.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "camera_depth_optical_frame","flap"));
      }else{
	ROS_WARN("cropResult size zero");
      }
    }
  public:


    tf::StampedTransform camera_transform;

    //activates when a new cloud is recived, convers it to ROS types and calls fusion with it as an argument
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
      if(debugLevel>1){ROS_INFO("%s: In cloud callback",nodeName.c_str());}

      pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
      pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
      if(debugLevel>2){
	ROS_INFO("Size in cloud_cb:");
	std::cout<< temp_cloud->size() <<std::endl;}
      computePose(temp_cloud);
    }

    //activates when CP publishes new target, saves to class private fields of class
    void vision_cb(const geometry_msgs::Pose& pose_msg){

    }

};
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


  //pull settings

  nh.getParam("fusion_settings/statOutlier_meanK", statOutlier_meanK);
  nh.getParam("fusion_settings/statOutlier_stdDev", statOutlier_stdDev);

  Fusion fusion;


  //tf listener stup
  //while(node.ok()){
  tf::TransformListener listener;
  try{
    listener.lookupTransform("/arm_camera","/camera_link",ros::Time(0),fusion.camera_transform);
  }catch(tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();

  }


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe (sTopic1.c_str(), 1, &Fusion::cloud_cb,&fusion);
  ros::Subscriber sub2 = nh.subscribe (sTopic2.c_str(), 1, &Fusion::vision_cb,&fusion);

  ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic1.c_str());
  ROS_INFO("%s: Subscribing to %s",nodeName.c_str(),sTopic2.c_str());

  // Create a ROS publisher for the output point cloud
  pose_pub = nh.advertise<geometry_msgs::Pose> (pTopic1, 1);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> (pTopic2, 1);

  //debug pub
  cloud_pub2=nh.advertise<sensor_msgs::PointCloud2> ("crop_zone",1);
  cloud_pub3=nh.advertise<sensor_msgs::PointCloud2> ("normals",1);	

  ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic1.c_str());
  ROS_INFO("%s: Publishing to %s",nodeName.c_str(),pTopic2.c_str());
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}		
