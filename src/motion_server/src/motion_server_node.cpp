/* AVRS-MQP: motion_msgs -g
 *  Maintainer: avrs.mqp@gmail.com
 *  Authors: Nikolas Gamarra, Ryan O'Brien
 */
//ROS 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

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
#include <moveit_msgs/GetCartesianPath.h>
//#include <moveit/planning_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <motion_msgs/MoveRobotAction.h>

//general
#include <math.h>


#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


class MoveRobotAction
{
  protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_msgs::MoveRobotAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_msgs::MoveRobotFeedback feedback_;
    motion_msgs::MoveRobotResult result_;

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

    void executeCB(const motion_msgs::MoveRobotGoalConstPtr &goal)
    {
      // helper variables
      ros::Rate r(1);
      bool success = true;


      //FEEDBACK NEEDS UPDATE
      /*    
	    feedback_.sequence.clear();
	    feedback_.sequence.push_back(0);
	    feedback_.sequence.push_back(1);
       */

      ROS_INFO("%s: ExcutingCB: X:%f Y:%f Z:%f ",action_name_.c_str(),goal->x,goal->y,goal->z);
      std::string group="manipulator";
      //create quaternion
      tf::Quaternion q_rot;
      tf::TransformListener listener;

      float roll, pitch, yaw, x, y, z;
      //pull all the values from goal
      roll=goal->roll;
      pitch=goal->pitch;
      yaw=goal->yaw;
      x=goal->x;
      y=goal->y;
      z=goal->z;
      group=goal->frame;
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
      moveit::planning_interface::MoveGroupInterface move_group(group);
      // Plan for robot to move to part

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;



      //ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      //moveit_msgs::DisplayTrajectory display_trajectory;


//TEST ZONE
      int mode=goal->mode;

      int visualize=0;
double status;
      moveit_msgs::RobotTrajectory trajectory;

      move_group.setPoseReferenceFrame(base_frame);

      if(mode==1){
	move_group.setPoseTarget(move_target);

	int status;
	move_group.move();
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      }else if(mode==2){

	std::vector< geometry_msgs::Pose > poses;


	//moveit_msgs::GetCartesianPath srv;
	//srv.request.wait_for_execution = true;

	//ros::ServiceClient executeKnownTrajectoryServiceClient = nh_.serviceClient<moveit_msgs::GetCartesianPathExecuteKnownTrajectory>("/execute_kinematic_path");


poses.push_back(poseEOAT);





	status=	move_group.computeCartesianPath(poses, 0.005, 0.0, trajectory, true);


	my_plan.trajectory_=trajectory;

	move_group.execute(my_plan);
	//	move_group.move();
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      }else if (mode==3){//queue a point draw DANGER ZONE 3 point touch

	std::vector< geometry_msgs::Pose > poses;


	//moveit_msgs::GetCartesianPath srv;
	//srv.request.wait_for_execution = true;

	//ros::ServiceClient executeKnownTrajectoryServiceClient = nh_.serviceClient<moveit_msgs::GetCartesianPathExecuteKnownTrajectory>("/execute_kinematic_path");


geometry_msgs::Pose poseClear;
geometry_msgs::Pose poseTouch;


quaternionTFToMsg(q_rot,poseClear.orientation);
quaternionTFToMsg(q_rot,poseTouch.orientation);

poseClear.position.x=goal->x;
poseClear.position.y=goal->y;
poseClear.position.z=.04;//magic

poseTouch.position.x=goal->x;
poseTouch.position.y=goal->y;
poseTouch.position.z=.015;//magic

ROS_WARN("MODE 3");



poses.push_back(poseClear);
poses.push_back(poseTouch);
poses.push_back(poseClear);




//poses.push_back(poseEOAT);





	status=	move_group.computeCartesianPath(poses, 0.005, 0.0, trajectory, true);


	my_plan.trajectory_=trajectory;

	move_group.execute(my_plan);
	//	move_group.move();
	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);





}

      feedback_.current=status;

      if(visualize=1){
	//display_publisher.publish(trajectory);

      }




      /*
	 if (status>0.0){
	 success=true;
	 }else {

	 success=false;
	 }
       */



      //____EX looped exicution with preemption
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	as_.setPreempted();
	success = false;
	move_group.stop();
      }
      // publish the feedback
      as_.publishFeedback(feedback_);

      //check success 
      if(success)
      {
	result_.statusCode =1;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
	// set the action state to succeeded
	as_.setSucceeded(result_);
      }else{//failure
	result_.statusCode=0;
	ROS_INFO("%s: Failure",action_name_.c_str());
	as_.setAborted(result_);
      }

    }

};


int main(int argc, char** argv)
{
  bool start=true;
  if(start)
    ROS_INFO("Server Running...");
  start=false;
  //ros::AsyncSpinner async_spinner(1);
  //async_spinner.start();
  ros::init(argc, argv, "motion");

  MoveRobotAction motion("motion");
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}
