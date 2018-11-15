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

int msgPend; //Number of messages waiting to be read
bool readFlag; //New msg will not be read unless flag is low. Set low after last msg is processed
void vehicle_callback(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");

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

      if (std::cout == "Tesla") {
	stream << line;
	msg.model= stream.str();

	std::cout << line;
	stream << line;
	msg.charger_type = stream.str();

	std::cout << line;
	msg.battery_charge = 50;

	std::cout << line;
	msg.charge_level = 3;
	--msgPend;

      }

      else if (std::cout == "Volt") {
	msg.model= "Volt";
	std::cout << line;
	msg.charger_type = "J1772";
//	std::cout << line;
//	msg.battery_charge = std::cout;
//	std::cout << line;
//	msg.charge_level = std::cout;
	--msgPend;
      }

      else if (std::cout == "Leaf") {
	msg.model= "Leaf";
	std::cout << line;
	msg.charger_type = "CHAdeMO";
//	std::cout << line;
//	msg.battery_charge = std::cout;
//	std::cout << line;
//	msg.charge_level = std::cout;
	--msgPend;
      }

    }

    msg_pub.publish(msg);
  }
}

  int main(int argc, char **argv) {

    ros::init(argc,argv, "Yun");
    ros::NodeHandle nh;

    system("ssh root@vehiclesim.local 'telnet localhost 6571' 2>&1 | tee coms_out.txt");
    ++msgPend;	

    msg_pub = nh.advertise<coms_msgs::Vehicle>("comsUplink", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
  

    ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), vehicle_callback);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    //}		

    //system("ssh root@vehiclesim.local 'telnet localhost 6571'");

    return 0;
  }

