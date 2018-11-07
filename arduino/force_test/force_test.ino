


/*
  //Combination of ros serial publisher and linear range finder programs with topic error reporting
  //Nikolas Gamarra
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <force_msgs/PointForce.h>
#include <force_msgs/PointForceArray.h>

ros::NodeHandle  nh;

force_msgs::PointForce ForceA_msg;
force_msgs::PointForce ForceB_msg;
force_msgs::PointForce ForceC_msg;
force_msgs::PointForceArray ForceArray_msg;

ros::Publisher pub_force( "force_data", &ForceArray_msg);

char frameid2[] = "/load_cell";


//definitions
#define loadCell1_pin 1//top
#define loadCell2_pin 2//clockwise
#define loadCell3_pin 3

void setup()
{
  nh.initNode();

  nh.advertise(pub_force);

  nh.getHardware()->setBaud(115200);



  ForceA_msg.header.frame_id = frameid2;
  ForceB_msg.header.frame_id = frameid2;
  ForceC_msg.header.frame_id = frameid2;

  //load cell a location
  ForceA_msg.xLoc = 0;
  ForceA_msg.yLoc = 2;
  ForceA_msg.zLoc = 0;
  //load cell b location
  ForceB_msg.xLoc = 1.732;
  ForceB_msg.yLoc = -1;
  ForceB_msg.zLoc = 0;
  //loat cell c location
  ForceC_msg.xLoc = -1.732;
  ForceC_msg.yLoc = -1;
  ForceC_msg.zLoc = 0;

}
float range = 0;

void loop()
{

  //load cell a location
  ForceA_msg.header.stamp =nh.now();
  ForceA_msg.xForce = 0;
  ForceA_msg.yForce = 0;
  ForceA_msg.zForce = -5;
  //load cell b location
    ForceB_msg.header.stamp =nh.now();
  ForceB_msg.xForce = 0;
  ForceB_msg.yForce = 0;
  ForceB_msg.zForce = -5;
  //loat cell c location
    ForceC_msg.header.stamp =nh.now();
  ForceC_msg.xForce = 0;
  ForceC_msg.yForce = 0;
  ForceC_msg.zForce = -5;

  ForceArray_msg.header.stamp =nh.now();
  ForceArray_msg.PointForces[0] = ForceA_msg;
  ForceArray_msg.PointForces[1] = ForceB_msg;
  ForceArray_msg.PointForces[2] = ForceC_msg;


  //ForceArray_msg.PointForces.push_back(ForceA_msg);
  //ForceArray_msg.PointForces.push_back(ForceB_msg);
  //ForceArray_msg.PointForces.push_back(ForceC_msg);

  pub_force.publish(&ForceArray_msg);

  nh.spinOnce();
  delay(1000);//1000
}
