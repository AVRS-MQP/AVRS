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
#include <std_msgs/Float32.h>

int debugLevel=2;//2 prints all things, 3 ignores mode topic

int myMode=0;


bool savedFlag=false;
bool savedFlag2=false;
double degTorad(double deg){
  return (deg*(M_PI/180));
}

class TFman{//the tf manipulator class #class is the tool of the enemy 
  private:
    geometry_msgs::Pose savedPose;//TODO boost share or make a class
    geometry_msgs::Pose savedPose2;//a pose for camer loc

    float myAngle=0;

    void setPose(tf::StampedTransform trans){

      tf::Vector3 orig=trans.getOrigin();

      savedPose.position.x=orig[0];
      savedPose.position.y=orig[1];
      savedPose.position.z=orig[2];

      tf::Quaternion rot=trans.getRotation();

      quaternionTFToMsg(rot,savedPose.orientation);
    }

    void setPose2(tf::StampedTransform trans){

      tf::Vector3 orig=trans.getOrigin();

      savedPose2.position.x=orig[0];
      savedPose2.position.y=orig[1];
      savedPose2.position.z=orig[2];

      tf::Quaternion rot=trans.getRotation();

      quaternionTFToMsg(rot,savedPose2.orientation);
    }


  public:
    void timerCallback(const ros::TimerEvent&){
      if(myMode==2){

	ROS_INFO("PUBLISHING");

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
	double closeClearDist=.02;
	double camClearDist = .3;
	double hingeX =-.1016;//meters
	double hingeZ=.0216;//meters
	double touchCorrection =0;
	double fixRoll =degTorad(0);//135
	double fixPitch =degTorad(0);//x rot
	double fixYaw = degTorad(0);//y rot
	double hingeAngle = degTorad(myAngle);//z rot
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
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_hinge", 		"flap_touching"));


	//clearance
	transf.setOrigin(tf::Vector3(0,0,-clearDist));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touching", 		"flap_clearance"));

	//close clearance
	transf.setOrigin(tf::Vector3(0,0,-closeClearDist));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touching", 		"flap_clearance_C"));


	//camera clearance
	transf.setOrigin(tf::Vector3(0,0,-camClearDist));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_saved", 		"cam_clearance"));

	//touching hole
	transf.setOrigin(tf::Vector3(0,0,.05));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_saved", 		"flap_hole_touching"));

	//clearance hole
	transf.setOrigin(tf::Vector3(0,0,-clearDist));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_saved", 		"flap_hole_clearance"));

	//exit flap clearance 
	transf.setOrigin(tf::Vector3(.1,0,-.1));
	q_rot = tf::createQuaternionFromRPY(0,degTorad(-45),0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_clearance","flap_clearance2"));

	//Left swipe end pos
	transf.setOrigin(tf::Vector3(0,0,-.1));
	q_rot = tf::createQuaternionFromRPY(0,0,0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_saved","swipe_end"));

	//Left swipe start pos
	transf.setOrigin(tf::Vector3(0,0,-.075));
	q_rot = tf::createQuaternionFromRPY(0,degTorad(-90),0);
	transf.setRotation(q_rot);
	br2.sendTransform(tf::StampedTransform(transf,ros::Time::now(), "flap_touching","swipe_start"));


      }
      if(myMode==4){

	double x=savedPose2.position.x;
	double y=savedPose2.position.y;
	double z=savedPose2.position.z;
	ROS_INFO("%f , %f , %f ",x, y ,z);
	tf::Quaternion savedQ;
	quaternionMsgToTF(savedPose2.orientation,savedQ);


	// the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(savedQ).getRPY(roll, pitch, yaw);

	// the found angles are written in a geometry_msgs::Vector3
	geometry_msgs::Vector3 rpy;

	//republish saved raw loc
	double NUMBER1;//placeholder
	double NUMBER2;
	static tf::TransformBroadcaster br2;
	tf::Transform transf;
	transf.setOrigin(tf::Vector3(x,y,z));
	transf.setRotation(savedQ);

	br2.sendTransform(tf::StampedTransform(transf, ros::Time::now(), "base_link","cam_saved"));






      }

    }

    bool executeCB(mode_msgs::Mode::Request &req, mode_msgs::Mode::Response &res){
      //	bool result= computePose();
      myMode=req.input2;
      ROS_WARN("TF-MODE: %i",myMode);

      if(myMode==1){//save it
	savedFlag=false;
	savePose();
      }else if (myMode==2){
      }else if (myMode==3){
	savedFlag2=false;
	savePose2();
      }else if(myMode==4){
	//publish it (handeled by timerCB and class mode)
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

void savePose2(){

  tf::TransformListener listener;


  tf::StampedTransform transform;

  if(savedFlag2==false){

    try{
      ros::Time now =ros::Time(0);

      listener.waitForTransform("/base_link","/cam",now,ros::Duration(6.0));

      listener.lookupTransform("/base_link", "/cam",  
	  now, transform);

      setPose2(transform);

      ROS_INFO("SAVING TF");
      savedFlag2=true;

    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}



void hingeCB(const std_msgs::Float32 msg){//save the toplic hinge angle val
  myAngle=msg.data;
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


  ros::Subscriber angleSub = nh.subscribe("hinge_angle",500,&TFman::hingeCB,&tfman);  
  // ros::spin();
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
}


