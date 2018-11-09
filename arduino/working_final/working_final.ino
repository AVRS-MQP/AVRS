/*
  //Combination of ros serial publisher and linear range finder programs with topic error reporting
  //Nikolas Gamarra
*/


//ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <force_msgs/LoadCellForces32.h>

//sensor includes
#include <Wire.h>
#include "Adafruit_VL6180X.h"

ros::NodeHandle  nh;

//make msgs
std_msgs::String str_msg;
sensor_msgs::Range range_msg;
force_msgs::LoadCellForces32 Force_msg;


//make publishers
ros::Publisher range_error("range_error", &str_msg);
ros::Publisher pub_force( "force_data", &Force_msg);
ros::Publisher pub_range( "range_data", &range_msg);

char frameid[] = "/ir_ranger";
char frameid2[] = "/load_cell";

//range finder setup
Adafruit_VL6180X vl = Adafruit_VL6180X();
String errorMSG = "default";

//definitions
#define loadCellA_pin 1//top
#define loadCellB_pin 2//clockwise
#define loadCellC_pin 3

void setup()
{
  nh.initNode();//initialize the ros node

  //advertise publishers
  nh.advertise(range_error);
  nh.advertise(pub_range);
  nh.advertise(pub_force);

  //use dedault
  //nh.getHardware()->setBaud(2000000);

  //setup msg headers
  Force_msg.header.frame_id = frameid2;

}
float range = 0;

void loop()
{

  //set load cell msg valuse
  Force_msg.header.stamp =nh.now();
  Force_msg.cellA = 5;
  Force_msg.cellB = 0;
  Force_msg.cellC = 10;

  pub_force.publish(&Force_msg);

  nh.spinOnce();
  delay(400);//1000
}
