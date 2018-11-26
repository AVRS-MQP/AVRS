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


	void coms_cb() {
	msg.headerstamp = ros::Time::now();
	coms_msgs::Station msg;
	//does things based on the message recieved
	//this will be handled in main

}


int main(int argc, char **argv) {

	ros::init(argc,argv, "Uplink");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe ("Yun", 1, coms_cb);
	ros::spin();

}
