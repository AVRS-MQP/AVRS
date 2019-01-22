#include "ros/ros.h"
#include "addints_msgs/AddTwoInts.h"

bool add(addints_msgs::AddTwoInts::Request  &req,
         addints_msgs::AddTwoInts::Response &res)
{
  res.Sum = req.A + req.B;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.A, (long int)req.B);
  ROS_INFO("sending back response: [%ld]", (long int)res.Sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
