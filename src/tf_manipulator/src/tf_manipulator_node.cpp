#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	tf::TransformListener listener;
	geometry_msgs::Pose savedPose;
	bool savedFlag=false;
	while (node.ok()){
		tf::StampedTransform transform;
		if(savedFlag==false){
			try{

				listener.lookupTransform("/base_link", "/flap_raw",  
						ros::Time(0), transform);


				tf::Vector3 orig=transform.getOrigin();


				savedPose.position.x=orig[0];
				savedPose.position.y=orig[1];
				savedPose.position.z=orig[2];

				tf::Quaternion rot=transform.getRotation();



				quaternionTFToMsg(rot,savedPose.orientation);


				//savedFlag=true;

			}
			catch (tf::TransformException ex){

				ROS_WARN("%s",ex.what());
			}
		}else{


			int x=savedPose.position.x;
			int y=savedPose.position.y;
			int z=savedPose.position.z;

			tf::Quaternion savedQ;
			quaternionMsgToTF(savedPose.orientation,savedQ);


			// the tf::Quaternion has a method to acess roll pitch and yaw
			double roll, pitch, yaw;
			tf::Matrix3x3(savedQ).getRPY(roll, pitch, yaw);

			// the found angles are written in a geometry_msgs::Vector3
			geometry_msgs::Vector3 rpy;
			//    rpy.x = roll;
			//   rpy.y = pitch;
			//  rpy.z = yaw;


			//republish saved raw loc

			float fixRoll =1.5707963;
			static tf::TransformBroadcaster br;
			tf::Transform transf;
			transf.setOrigin(tf::Vector3(x,y,z));
			transf.setRotation(savedQ);
			br.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "camera_depth_optical_frame","flap_raw"));



			//publish roll fixed

			tf::Transform rollFixed;
			rollFixed.setOrigin(tf::Vector3(0,0,0));
			tf::Quaternion q_rot = tf::createQuaternionFromRPY(roll,pitch,yaw+fixRoll);
			rollFixed.setRotation(q_rot);
			br.sendTransform(tf::StampedTransform(rollFixed,ros::Time::now(), "flap_raw","flap_fixed"));

			//publish hinge

			tf::Transform hinge;
			hinge.setOrigin(tf::Vector3(.1,0,.1));
			q_rot = tf::createQuaternionFromRPY(0,0,0);
			hinge.setRotation(q_rot);
			br.sendTransform(tf::StampedTransform(hinge,ros::Time::now(), "flap_fixed","flap_hinge"));





		}


	}
	return 0;
};
