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
#include "std_msgs/String.h"
#include "mode_msgs/Mode.h"
int debugLevel=2;//2 prints all things, 3 ignores mode topic

int myMode=0;


bool savedFlag=false;
double degTorad(double deg){
  return (deg*(M_PI/180));
}

class TFman{//the tf manipulator class #class is the tool of the enemy 
  private:
    geometry_msgs::Pose savedPose;//TODO boost share or make a class



    void setPose(tf::StampedTransform trans){




      tf::Vector3 orig=trans.getOrigin();


      savedPose.position.x=orig[0];
      savedPose.position.y=orig[1];
      savedPose.position.z=orig[2];

      tf::Quaternion rot=trans.getRotation();


      quaternionTFToMsg(rot,savedPose.orientation);


    }
  public:
    void timerCallback(const ros::TimerEvent&){
if(myMode==2){

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
      double fixRoll =degTorad(0);//135
      double fixPitch =degTorad(0);//x rot
      double fixYaw = degTorad(0);//y rot
      double hingeAngle = degTorad(90);//z rot
      static tf::TransformBroadcaster br2;
      tf::Transform transf;
      transf.setOrigin(tf::Vector3(x,y,z));
      transf.setRotation(savedQ);
      br2.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "base_link","flap_saved"));



      //publish roll fixed

      transf.setOrigin(tf::Vector3(0,0,0));
      tf::Quaternion q_rot = tf::createQuaternionFromRPY(fixRoll,fixPitch,fixYaw);
      transf.setRotation(q_rot);
      br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_saved","flap_fixed"));

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
      br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touching","flap_clearance"));

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

}

    bool executeCB(mode_msgs::Mode::Request &req, mode_msgs::Mode::Response &res){
      //	bool result= computePose();
      myMode=req.input2;
  ROS_WARN("MODE: %i",myMode);
      if(myMode==1){

	savedFlag=false;
      }


  if(myMode==1){//save it
    savePose();
  }else if (myMode==2){//publish it
  }
      //res.done=result;
      return true;	
    }


    void savePose(){

      tf::TransformListener listener;


      tf::StampedTransform transform;

      if(savedFlag==false){

	try{
	  ros::Time now =ros::Time(0);

	  listener.waitForTransform("/base_link","/flap_raw",now,ros::Duration(6.0));

	  listener.lookupTransform("/base_link", "/flap_raw",  
	      now, transform);

	  setPose(transform);

	  ROS_INFO("SAVING TF");
	  savedFlag=true;

	}
	catch (tf::TransformException ex){
	  ROS_WARN("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
    }
}





};

static std::string nodeName("temp_name");

int main(int argc, char** argv){

  ros::init(argc, argv, nodeName);


  ros::NodeHandle nh;

  nodeName = ros::this_node::getName();//Update name

  TFman tfman;
//ros::Rate r(1);
//while(ros::ok()){

  ros::ServiceServer service = nh.advertiseService("transform",&TFman::executeCB,&tfman);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), &TFman::timerCallback,&tfman);
  
 // ros::spin();
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}


