#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <fstream>

int main () {

	string line;
	ofstream myFile;
	myFile.open ("coms_out", ios::in | ios::ate);

	if (myfile.is_open()) { 
		while (getline (myfile,line)) { //reads through the opened text file line by line
			cout << line << '/n';
			
			//cout values are place holder, needs to search for specific lines that are
			//prewritten on the Yun side and then publish a packet to ROS based on that
			if (cout == "Tesla") {
			
			}
			
			else if (cout == "Volt") {

			}

			else if (cout == "Leaf") {

			}

		}
	}		

/*
int main(int argc, char **argv) {

ros::init(argc,argv, "Yun");
ros::NodeHandle n;
*/

}
