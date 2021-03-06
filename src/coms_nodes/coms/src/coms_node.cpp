/* AVRS-MQP: abb_motion_actionserver
*  Maintainer: avrs.mqp@gmail.com
*  Authors: Nikolas Gamarra, Ryan O'Brien
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <coms_msgs/Vehicle.h>
#include <coms_msgs/Station.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>


ros::Publisher msg_pub;
int count = 0;

int msgPend; //Number of messages waiting to be read
bool readFlag; //New msg will not be read unless flag is low. Set low after last msg is processed

void vehicle_callback(const ros::TimerEvent&)
{
  ROS_INFO("Vehicle callback triggered");

  std::string line; //Reads each line and marks line as read
  std::ostringstream stream; //Can hold lines from text file but doesn't mark read. Can be converted 				  //to string
  std::fstream myFile;
  myFile.open ("coms_out", std::ios::in | std::ios::ate);


  coms_msgs::Vehicle msg;
  msg.headerstamp = ros::Time::now();

  if (myFile.is_open()) {
    while (getline (myFile,line)) { //reads through the opened text file line by line
      // std::cout << line << '/n';
      std::cout << line;

			stream << line;
      msg.model= stream.str();

			// Only part that matters right now
			std::cout << line;
      stream << line;
      msg.charger_type = stream.str();


			//currently unused

      if (std::cout == "Tesla") {
        stream << line;
        msg.model= stream.str();

        std::cout << line;
        stream << line;
        msg.charger_type = stream.str();

        //std::cout << line;
        //stream << line;
       //msg.battery_charge = stream.str();

        std::cout << line;
        msg.charge_level = 3;
        --msgPend;

        msg.flap_auto_open = true;

      }

    
    } //while

    msg_pub.publish(msg);
  }
}


int main(int argc, char **argv) {

  ros::init(argc,argv, "Yun");
  ros::NodeHandle nh;

  //Connect To Yun
  system("ssh root@vehiclesim.local 'telnet localhost 6571' 2>&1 | tee coms_out.txt");
  ++msgPend;
  //ros::Duration(4).sleep(); //pause to make sure Yun is connected
  //system("arduino"); //enter password to establish connection

  //ros::Subscriber sub = nh.subscribe ("comsUplink", 1, vehicle_callback);

  ros::Publisher pub = nh.advertise<coms_msgs::Vehicle>("vehicle_data", 1);
  msg_pub = nh.advertise<coms_msgs::Vehicle>("comsUplink", 1);
  ros::Rate loop_rate(5);


  ros::Timer timer1 = nh.createTimer(ros::Duration(1), vehicle_callback);

  ros::spin();
  loop_rate.sleep();
  ++count;


  return 0;
}
