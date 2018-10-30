#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {

ros::init(argc,argv, "Yun");
ros::NodeHandle n;

system("ssh root@vehiclesim.local 'telnet localhost 6571'");
     
return 0;
}

