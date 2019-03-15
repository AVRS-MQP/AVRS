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

ros::Publisher msg_pub;
std::string charger;
bool readFlag = 0;

void coms_cb() {
	readFlag = 1;
	//charger = coms_msgs::Vehicle.charger_type;

}

void read_msg() {

	if (readFlag) { //If message has been received then return the charger type
		readFlag = 0;
		//return charger;	
	}
	
	else {
		ROS_INFO("Searching for vehicle");
	}

}


int main(int argc, char **argv) {

	ros::init(argc,argv, "coms_service");
	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("vehicle_uplink", read_msg);

	ros::Subscriber sub = nh.subscribe ("Yun", 1, coms_cb);
	ros::spin();

}
