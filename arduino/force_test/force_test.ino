/*
  //Combination of ros serial publisher and linear range finder programs with topic error reporting
  //Nikolas Gamarra
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <force_msgs/LoadCellForces32.h>

ros::NodeHandle  nh;

force_msgs::LoadCellForces32 Force_msg;


ros::Publisher pub_force( "force_data", &Force_msg);

char frameid2[] = "/load_cell";


//definitions
#define loadCellA_pin 1//top
#define loadCellB_pin 2//clockwise
#define loadCellC_pin 3

void setup()
{
  nh.initNode();

  nh.advertise(pub_force);

  //use dedault
  //nh.getHardware()->setBaud(2000000);



  Force_msg.header.frame_id = frameid2;

}
float range = 0;

void loop()
{

  //set load cell msg valuse
  Force_msg.header.stamp = nh.now();
  Force_msg.cellA = 5;
  Force_msg.cellB = 0;
  Force_msg.cellC = 10;

  pub_force.publish(&Force_msg);

  nh.spinOnce();
  delay(400);//1000
}
