#include "ros/ros.h"
#include "std_msgs/String.h"

#include <coms_msgs/vehicle.h>
#include <coms_msgs/station.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>

ros::Publisher msg_pub;

void vehicle_callback(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");

  std::string line;
  std::fstream myFile;
  myFile.open ("coms_out", std::ios::in | std::ios::ate);


  coms_msgs::vehicle msg;
  msg.headerstamp = ros::Time::now();

  if (myFile.is_open()) { 
    while (getline (myFile,line)) { //reads through the opened text file line by line
     // std::cout << line << '/n';
	std::cout << line;

      //std::cout values are place holder, needs to search for specific lines that are
      //prewritten on the Yun side and then publish a packet to ROS based on that
      if (std::cout == "Tesla") {
	msg.model= "Tesla";
	//std::cout << line;
	//msg.charger_type = std::cout;
//	std::cout << line;
//	msg.battery_charge = std::cout;
//	std::cout << line;
//	msg.charge_level = std::cout;
//

      }

      else if (std::cout == "Volt") {

      }

      else if (std::cout == "Leaf") {

      }

    }

    msg_pub.publish(msg);
  }
}

  int main(int argc, char **argv) {

    ros::init(argc,argv, "Yun");
    ros::NodeHandle n;

    system("ssh root@vehiclesim.local 'telnet localhost 6571' 2>&1 | tee coms_out.txt");

    msg_pub = n.advertise<coms_msgs::vehicle>("comsUplink", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
  

    ros::Timer timer1 = n.createTimer(ros::Duration(0.1), vehicle_callback);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    //}		

    //system("ssh root@vehiclesim.local 'telnet localhost 6571'");

    return 0;
  }

