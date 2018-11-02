#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {

ros::init(argc,argv, "Yun");
ros::NodeHandle n;

system("ssh root@vehiclesim.local 'telnet localhost 6571' 2>&1 | tee coms_out.txt");

ros::Publisher chatter_pub = n.advertise<std_msgs::String>("comsUplink", 1000);
ros::Rate loop_rate(10);

int count = 0;

//while(ros::ok()) {

	std_msgs::String msg;
	std::stringstream ss;
	ss << "connected to coms" << count;
	msg.data = ss.str();

	ROS_INFO("%s", msg.data.c_str());

	chatter_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
//}		

//system("ssh root@vehiclesim.local 'telnet localhost 6571'");
     
return 0;
}

