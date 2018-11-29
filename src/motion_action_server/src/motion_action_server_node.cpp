/* AVRS-MQP: motion_action_server_msgs -g
 *  Maintainer: avrs.mqp@gmail.com
 *  Authors: Nikolas Gamarra, Ryan O'Brien
 */
//ROS 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_action_server_msgs/MoveRobotAction.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
//MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//#include <moveit/planning_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
//general
#include <math.h>

class MoveRobotAction
{
  protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_action_server_msgs::MoveRobotAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_action_server_msgs::MoveRobotFeedback feedback_;
    motion_action_server_msgs::MoveRobotResult result_;

  public:

    MoveRobotAction(std::string name) :
      as_(nh_, name, boost::bind(&MoveRobotAction::executeCB, this, _1), false),
      action_name_(name)
  {
    as_.start();
  }

    ~MoveRobotAction(void)
    {
    }

    void executeCB(const motion_action_server_msgs::MoveRobotGoalConstPtr &goal)
    {
      // helper variables
      ros::Rate r(1);
      bool success = true;

      // push_back the seeds for the motion sequence

      //FEEDBACK NEEDS UPDATE
      /*    
	    feedback_.sequence.clear();
	    feedback_.sequence.push_back(0);
	    feedback_.sequence.push_back(1);
       */
      // publish info to the console for the user
      //    ROS_INFO("%s: Executing, creating motion sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);


      ROS_INFO("%s: ExcutingCB: X:%d Y:%d Z:%d ",action_name_.c_str(),goal->x,goal->y,goal->z);

      //create quaternion
      tf::Quaternion q_rot;
      tf::TransformListener listener;

      float roll, pitch, yaw, x, y, z;
      //pull all the values from goal
      roll=goal->roll;//TODO
      pitch=goal->pitch;
      yaw=goal->yaw;
      x=goal->x;
      y=goal->y;
      z=goal->z;
      //convert deg to rad
      roll=roll*(M_PI/180);
      pitch=pitch*(M_PI/180);
      yaw=yaw*(M_PI/180);

      //create and fill pose	
      q_rot = tf::createQuaternionFromRPY(roll, pitch, yaw);//roll(x), pitch(y), yaw(z),
      geometry_msgs::Pose poseEOAT;
      quaternionTFToMsg(q_rot,poseEOAT.orientation);
      poseEOAT.position.x= x;
      poseEOAT.position.y= y;
      poseEOAT.position.z= z;
      //setup move_group and run 
      std::string base_frame = "/base_link";

      geometry_msgs::Pose move_target = poseEOAT;
      moveit::planning_interface::MoveGroupInterface move_group("manipulator");
      // Plan for robot to move to part
      move_group.setPoseReferenceFrame(base_frame);
      move_group.setPoseTarget(move_target);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
int status;
      move_group.move();
      //status=MoveItErrorCode(move_group.asyncExecute(my_plan));
      //ROS_INFO(status);
      // start executing the action
      /*
      //____EX looped exicution with preemption
      for(int i=1; i<=goal->order; i++)
      {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
      }
       */
      if(success)
      {
	result_.location = feedback_.location;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
	// set the action state to succeeded
	as_.setSucceeded(result_);
      }
    }


};


int main(int argc, char** argv)
{
  //ros::AsyncSpinner async_spinner(1);
  //async_spinner.start();
  ros::init(argc, argv, "motion");

  MoveRobotAction motion("motion");
    ros::spin();
  //ros::waitForShutdown();

  return 0;
}
