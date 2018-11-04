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

ros::Publisher cl0_pub;
ros::Publisher cl1_pub;
ros::Publisher cl2_pub;
ros::Publisher cl3_pub;
ros::Publisher cl4_pub;
ros::Publisher cl5_pub;
ros::Publisher cl6_pub;
ros::Publisher cl7_pub;
ros::Publisher cl8_pub;
ros::Publisher cl9_pub;

int debugLevel =2;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	ROS_DEBUG_COND(debugLevel==2,"%s: In Callback",nodeName.c_str());

	// Create a container for the data and filtered data.
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


	if(mode==0){//passThrough 
		ROS_INFO("PCP: passThrough");
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

		sensor_msgs::PointCloud2 output;//create output container
		pcl::PCLPointCloud2 temp_output;//create PCLPC2
		pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
		pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
		pc2_pub.publish (output);// Publish the data.

	}
	if(mode==1){//outlerRemoval
		ROS_INFO("PCP: outlierRemoval");
		// Create a container for the data and filtered data.
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

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
	if (mode==2){//transform
		ROS_INFO("PCP: transform");	
		ros::Time begin = ros::Time::now();

		// Create a container for the data.
		sensor_msgs::PointCloud2 output2;

		//float theta =-M_PI/2;//DELETE

		pcl::PointCloud<pcl::PointNormal> temp_cloud2;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << xTrans,yTrans,zTrans;//translate xyz
		transform.rotate(Eigen::AngleAxisf(alphaf, Eigen::Vector3f::UnitX()));//rotate abt x
		transform.rotate(Eigen::AngleAxisf(betaf, Eigen::Vector3f::UnitY()));//rotate abt y
		//transform.rotate(Eigen::AngleAxisf(gammaf, Eigen::Vector3f::UnitZ()));//rotate abt z

		// Print the transformation
		printf ("\nTransform Matrix:\n");
		std::cout << transform.matrix() << std::endl;

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointNormal> ());
		pcl::transformPointCloudWithNormals(*temp_cloud,*cloud_transformed,transform);

		if(mode==2){
			pcl::PCLPointCloud2 temp_output;//create PCLPC2
			pcl::toPCLPointCloud2(*cloud_transformed,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
			pcl_conversions::fromPCL(temp_output,output2);//convert to ROS data typeos::Subscriber sub2 = nh.subscribe(sTopic2.c_str(), 1, message_cb);
			// Publish the data.
			pc2_pub.publish (output2);
		}

	}
	if (mode==3){//don
		ROS_INFO("PCP: DoN");
		ros::Time begin = ros::Time::now();
		//-----don

		//NIKO IN
		pcl::PCDWriter writer;
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2

		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<int> indicies;
		pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		pcl::removeNaNFromPointCloud(*in_cloud, *cloud, indicies);

		//visualization_msgs::MarkerArray markerArray;
		//visualization_msgs::Marker marker;
		cloud->is_dense = false; //fix non dense clouds getting rejected

		// Create a search tree, use KDTreee for non-organized data.
		pcl::search::Search<pcl::PointXYZ>::Ptr tree;
		if (cloud->isOrganized ())
		{
			tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
		}
		else
		{
			tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
		}
		// Set the input pointcloud for the search tree
		tree->setInputCloud (cloud);

		if (scale1 >= scale2)
		{
			ROS_INFO("Error: Large scale must be > small scale!");
			exit (EXIT_FAILURE);
		}
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indicies);

		// Compute normals using both small and large scales at each point
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
		ne.setInputCloud (cloud);
		ne.setSearchMethod (tree);

		//   NOTE: setting viewpoint is very important, so that we can ensure
		//   normals are all pointed in the same direction!

		ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

		// calculate normals with the small scale
		ROS_INFO( "Calculating normals for scale...");
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

		ne.setRadiusSearch (scale1);
		ne.compute (*normals_small_scale);

		// calculate normals with the large scale
		ROS_INFO( "Calculating normals for scale..." );
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

		ne.setRadiusSearch (scale2);
		ne.compute (*normals_large_scale);

		// Create output cloud for DoN results
		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

		ROS_INFO( "Calculating DoN... ");
		// Create DoN operator
		pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
		don.setInputCloud (cloud);
		don.setNormalScaleLarge (normals_large_scale);
		don.setNormalScaleSmall (normals_small_scale);

		if (!don.initCompute ())
		{
			std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
			exit (EXIT_FAILURE);
		}


		// Compute DoN
		don.computeFeature (*doncloud);

		// Save DoN features
		//pcl::PCDWriter writer;//moved for scope
		writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

		// Filter by magnitude
		ROS_INFO( "Filtering out DoN mag <= %f ",threshold );

		// Build the condition for filtering
		pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
				new pcl::ConditionOr<pcl::PointNormal> ()
				);
		range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
					new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
				);
		// Build the filter
		pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
		condrem.setInputCloud (doncloud);

		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

		// Apply filter
		condrem.filter (*doncloud_filtered);

		doncloud = doncloud_filtered;

		// Save filtered output
		ROS_INFO( "Filtered Pointcloud: %i data points.",doncloud->points.size());

		writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

		// Filter by magnitude
		ROS_INFO( "Clustering using EuclideanClusterExtraction with tolerance <= %f" ,segradius);

		pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
		segtree->setInputCloud (doncloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

		ec.setClusterTolerance (segradius);
		ec.setMinClusterSize (minClusterSize);//SETTING old was 50
		ec.setMaxClusterSize (maxClusterSize);//SETTING old was 100000
		ec.setSearchMethod (segtree);
		ec.setInputCloud (doncloud);
		ec.extract (cluster_indices);
		int cloudNum=0;
		int j = 0;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				cloud_cluster_don->points.push_back (doncloud->points[*pit]);
			}

			cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
			cloud_cluster_don->height = 1;
			cloud_cluster_don->is_dense = true;

			std::cout << "ObjDetDoN: PointCloud representing the Cluster: " << cloud_cluster_don->points.size ()<< " data points."<< std::endl;

			//publish mini clusters
			sensor_msgs::PointCloud2 output;//create output container
			pcl::PCLPointCloud2 temp_output;//create PCLPC2

			pcl::PointCloud<pcl::PointXYZ>::Ptr temX (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

			//Cut a box
			pcl::PointNormal pMin,pMax;
			pcl::getMinMax3D (*cloud_cluster_don,pMin,pMax);
			float xMaxf=pMax.x;
			float xMinf=pMin.x;
			float yMaxf=pMax.y;
			float yMinf=pMin.y;
			float zMaxf=pMax.z;
			float zMinf=pMin.z;

			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ

			pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
			pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::PassThrough<pcl::PointXYZ> ptfilter (true); 
			ptfilter.setInputCloud (temp_cloud);
			ptfilter.setFilterFieldName ("x");
			ptfilter.setFilterLimits (xMinf, xMaxf);
			ptfilter.filter (*cloud_filtered_x);

			ptfilter.setInputCloud(cloud_filtered_x);
			ptfilter.setFilterFieldName ("y");
			ptfilter.setFilterLimits (yMinf,yMaxf);
			ptfilter.filter (*cloud_filtered_xy);

			ptfilter.setInputCloud(cloud_filtered_xy);
			ptfilter.setFilterFieldName ("z");
			ptfilter.setFilterLimits (zMinf,zMaxf);//SETTING
			ptfilter.setNegative (false);
			ptfilter.filter (*cloud_filtered_xyz);

			pcl::toPCLPointCloud2(*cloud_filtered_xyz,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
			pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
			/*
			//box
			float xScale=(pMax.x-pMin.x);
			float yScale=(pMax.y-pMin.y);
			float zScale=(pMax.z-pMin.z);
			abs(xScale);
			abs(yScale);
			abs(zScale);

			float xLoc= ((pMax.x-pMin.x)/2)+pMin.x;
			float yLoc= ((pMax.y-pMin.y)/2)+pMin.y;
			float zLoc= ((pMax.z-pMin.z)/2)+pMin.z;
			 */			
			//rviz_visual_tools::RvizVisualTools.deleteAllMarkers();

			//TODO MAKE MARKER BUILDER DO GAS PUCK
			//marker=markerBuilder(j,loc,scale,min,max,0,cloud_cluster_don->width);
			//markerArray.markers.push_back(marker);

			//custom msg garbage TODO REPLACE WITH POSE MSG

			ROS_INFO("AT START OF CL SWITCH");
			switch(cloudNum){
				case 0:
					ROS_INFO("CL0");
					cl0_pub.publish (output);
					break;
				case 1:
					ROS_INFO("CL1");
					cl1_pub.publish (output);
					break;
				case 2:
					cl2_pub.publish (output);
					break;
				case 3:
					cl3_pub.publish (output);
					break;
				case 4:
					cl4_pub.publish (output);
					break;
				case 5:
					cl5_pub.publish (output);
					break;
				case 6:
					cl6_pub.publish (output);
					break;
				case 7:
					cl7_pub.publish (output);
					break;
				case 8:
					cl8_pub.publish (output);
					break;
				case 9:
					cl9_pub.publish (output);
					break;
				default:
					ROS_INFO("ERR: More clusters than available pc2 topics.");
					//cloudNum=-1;
					break;
			}
			cloudNum++;
		}
		if(mode==1){
		}


		//TODO fix vis pub for single marker or keep array and add only one
		//also add pose not marker senter at pose not cloud
		//vis_pub.publish(markerArray);



		markerID=cloudNum;


		ros::Time end =ros::Time::now();
		float duration=end.toSec() - begin.toSec();
		ROS_INFO("%s: Out of callback. Duration: %f",nodeName.c_str(),duration);
		//ROS_INFO(duration);
		//=====don

	}
	if(mode==4){
		int m2=1;

		//NEW CONVERSION
		pcl::PCLPointCloud2 pcl_pc2;//create PCLPC2
		pcl_conversions::toPCL(*cloud_msg,pcl_pc2);//convert ROSPC2 to PCLPC2
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);//create PCLXYZ
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);//convert PCLPC2 to PCLXYZ
		//euclidian cluster extraxion
		//these were moved here for scope of mode
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDWriter writer;
		if(m2==1){
			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::VoxelGrid<pcl::PointXYZ> vg;
			vg.setInputCloud (temp_cloud);
			vg.setLeafSize (leafSize, leafSize, leafSize);//default (0.01f, 0.01f, 0.01f)//SETTING
			vg.filter (*cloud_filtered);
			std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
			// Create the segmentation object for the planar model and set all the parameters
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (maxIterations);//SETTING
			seg.setDistanceThreshold (distanceThreshold);//SETTING

			int i=0, nr_points = (int) cloud_filtered->points.size ();
			while (cloud_filtered->points.size () > 0.3 * nr_points)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud (cloud_filtered);
				seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0)
				{
					std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
					break;
				}
				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud (cloud_filtered);
				extract.setIndices (inliers);
				extract.setNegative (false);
				// Get the points associated with the planar surface
				extract.filter (*cloud_plane);
				std::cout << "ObjDetEuc: PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

				//add missing cloud_f //NI
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_f);
				*cloud_filtered = *cloud_f;
			}
		}else if(m2==2||m2==3){
			cloud_filtered=temp_cloud;
		}
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_filtered);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (clusterTollerance); // 2cm//SETTING
		ec.setMinClusterSize (minClusterSize);//SETTING
		ec.setMaxClusterSize (maxClusterSize);//SETTING
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);
		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "ObjDetEuc: PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "cloud_cluster_" << j << ".pcd";
			writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
			j++;
			pcl::CentroidPoint<pcl::PointXYZ> centroid;//add cluster centroid to array

			for (std::vector<int>::const_iterator pit2 = it->indices.begin (); pit2 != it->indices.end (); ++pit2){
				centroid.add(pcl::PointXYZ(cloud_filtered->points[*pit2].x,cloud_filtered->points[*pit2].y,cloud_filtered->points[*pit2].z));
			}
			pcl::PointXYZ c1;
			centroid.get (c1);

			//convert back
			sensor_msgs::PointCloud2 output;//create output container
			pcl::PCLPointCloud2 temp_output;//create PCLPC2
			pcl::toPCLPointCloud2(*cloud_filtered,temp_output);//convert from PCLXYZ to PCLPC2 must be pointer input
			pcl_conversions::fromPCL(temp_output,output);//convert to ROS data type
			pc2_pub.publish (output);// Publish the data.



		}
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
	}else if(myMode=="4"||myMode=="euc"||myMode=="E"){
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
	//cluster publishers
	cl0_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl0", 1);
	cl1_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl1", 1);
	cl2_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl2", 1);
	cl3_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl3", 1);
	cl4_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl4", 1);
	cl5_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl5", 1);
	cl6_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl6", 1);
	cl7_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl7", 1);
	cl8_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl8", 1);
	cl9_pub = nh.advertise<sensor_msgs::PointCloud2> ("cl9", 1);
	ros::spin();
}		

