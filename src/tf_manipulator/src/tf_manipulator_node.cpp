#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;
geometry_msgs::Pose savedPose;

  while (node.ok()){
    tf::StampedTransform transform;
    try{


tf::Transform transform;
      listener.lookupTransform("/base_link", "/flap_raw",  
                               ros::Time(0), transform);

	savedPose.position=transform.getOrigin();
	savedPose.orientation=transform.getRotation();
    }
    catch (tf::TransformException ex){

      ROS_WARN("%s",ex.what());
	ROS_INFO("Using last pose information to publish new poses");


        float fixRoll =1.5707963;
 static tf::TransformBroadcaster br;
        tf::Transform transf;
        transf.setOrigin(tf::Vector3(x,y,z));
        transf.setRotation(q_rot);
        br.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "camera_depth_optical_frame","flap_raw"));

        float clearDist=.2;
        tf::Transform clear;
        tf::Quaternion q;
        float fixRoll =1.5707963;
        q.setRPY(0,0,yaw+fixRoll);
        clear.setOrigin(tf::Vector3(0,0,-clearDist));
        clear.setRotation(q);

        br.sendTransform(tf::StampedTransform(clear,ros::Time::now(),"flap_raw","flap_clear"));


          tf::Transform touch;
          q.setRPY(0,0,0);
          touch.setOrigin(tf::Vector3(0,0,clearDist));
          touch.setRotation(q);


          br.sendTransform(tf::StampedTransform(touch,ros::Time::now(),"flap_clear","flap_touch"));

   static tf::TransformBroadcaster br;
        tf::Transform transf;
        transf.setOrigin(savedPose.position);
        transf.setRotation(savedPose.orientation.roll,savedPosition.orientation.pitch,savedPose.orientation.yaw+fixRoll);
        br.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "camera_depth_optical_frame","flap_fixed"));

        tf::Transform clear;
        tf::Quaternion q;
        q.setRPY(0,0,0);
        clear.setOrigin(tf::Vector3(0,0,-clearDist));
        clear.setRotation(q);

        br.sendTransform(tf::StampedTransform(clear,ros::Time::now(),"flap_raw","flap_clear"));


          tf::Transform touch;
          q.setRPY(0,0,0);
          touch.setOrigin(tf::Vector3(0,0,clearDist));
          touch.setRotation(q);


          br.sendTransform(tf::StampedTransform(touch,ros::Time::now(),"flap_clear","flap_touch"));

	
	}


  }
  return 0;
};
