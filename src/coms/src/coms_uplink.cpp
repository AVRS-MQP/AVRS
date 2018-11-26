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


void coms_cb() {


}


int main(int argc, char **argv) {


	ros::init(argc,argv, "Uplink");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe ("Yun", 1, coms_cb);
	ros::spin();

}
