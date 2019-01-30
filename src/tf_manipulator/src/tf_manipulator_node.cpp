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
double degTorad(double deg){
	return (deg*(M_PI/180));
}

	geometry_msgs::Pose savedPose;//TODO boost share
void timerCallback(const ros::TimerEvent&){


ROS_INFO("PUBLISHING");//TODO

			double x=savedPose.position.x;
			double y=savedPose.position.y;
			double z=savedPose.position.z;
ROS_INFO("%f , %f , %f ",x, y ,z);
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

double clearDist=.2;
double hingeX =-.1016;//meters
double hingeZ=.0216;//meters
double touchCorrection =0;
			double fixRoll =degTorad(135.0);
double hingeAngle = degTorad(90.0);
			static tf::TransformBroadcaster br2;
			tf::Transform transf;
			transf.setOrigin(tf::Vector3(x,y,z));
			transf.setRotation(savedQ);
			br2.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "base_link","flap_raw"));



			//publish roll fixed

			transf.setOrigin(tf::Vector3(0,0,0));
			tf::Quaternion q_rot = tf::createQuaternionFromRPY(0,0,fixRoll);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_raw","flap_fixed"));

			//publish hinge
			transf.setOrigin(tf::Vector3(hingeX,0,hingeZ));
			q_rot = tf::createQuaternionFromRPY(0,hingeAngle,0);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_fixed","flap_hinge"));


			//touching
			transf.setOrigin(tf::Vector3(-hingeX,0,-hingeZ+touchCorrection));
			q_rot = tf::createQuaternionFromRPY(0,0,0);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_hinge","flap_touching"));


			//clearance
			transf.setOrigin(tf::Vector3(0,0,-clearDist));
			q_rot = tf::createQuaternionFromRPY(0,0,0);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touchingA","flap_clearance"));

/*
			//touching
			transf.setOrigin(tf::Vector3(-hingeX,0,-hingeZ+touchCorrection));
			q_rot = tf::createQuaternionFromRPY(0,0,0);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_hinge","flap_touchingB"));

			//clearance
			transf.setOrigin(tf::Vector3(0,0,-clearDist));
			q_rot = tf::createQuaternionFromRPY(0,0,0);
			transf.setRotation(q_rot);
			br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touchingA","flap_clearanceB"));

*/

}
int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle nh;




	
	tf::TransformListener listener;
	bool savedFlag=false;
	while (nh.ok()){
		tf::StampedTransform transform;
		if(savedFlag==false){
			try{
				ros::Time now =ros::Time(0);
				
				listener.waitForTransform("/base_link","/flap_raw",now,ros::Duration(3.0));

				listener.lookupTransform("/base_link", "/flap_raw",  
						now, transform);


				tf::Vector3 orig=transform.getOrigin();


				savedPose.position.x=orig[0];
				savedPose.position.y=orig[1];
				savedPose.position.z=orig[2];

				tf::Quaternion rot=transform.getRotation();



				quaternionTFToMsg(rot,savedPose.orientation);

				ROS_INFO("SAVING TF");
				savedFlag=true;

			}
			catch (tf::TransformException ex){
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
		}else{
ROS_INFO("calling");
ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);



ros::spin();

		}


	}
	return 0;
};
