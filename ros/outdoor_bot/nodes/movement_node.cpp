// movement control for outdoor bot
// controls a robot from speech or keyboard commands
/*
w	x	y	z	Description
1	0	0	0	Identity quaternion, no rotation
0	1	0	0	180° turn around X axis
0	0	1	0	180° turn around Y axis
0	0	0	1	180° turn around Z axis
sqrt(0.5)	sqrt(0.5)	0	0	90° rotation around X axis
sqrt(0.5)	0	sqrt(0.5)	0	90° rotation around Y axis
sqrt(0.5)	0	0	sqrt(0.5)	90° rotation around Z axis
sqrt(0.5)	-sqrt(0.5)	0	0	-90° rotation around X axis
sqrt(0.5)	0	-sqrt(0.5)	0	-90° rotation around Y axis
sqrt(0.5)	0	0	-sqrt(0.5)	-90° rotation around Z axis

*/

#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include "outdoor_bot/movement_msg.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class movementControl
{
private:

   ros::NodeHandle nh_;
   ros::Publisher cmd_vel_pub_;
   // we subscribe motion and keyboard commands publishers
   ros::Subscriber keyboardCommandsSub_, motionCommandsSub_;

   // create the action client to send goals to move_base
   // true causes the client to spin its own thread
   MoveBaseClient *ac_;

   int distance_, angle_, map_or_bot_;
   int quatX_, quatY_, quatZ_, quatW_;
   std::string command_;

public:
  movementControl(ros::NodeHandle &nh)
   :  nh_(nh)
{
   distance_ = 0;
   angle_ = 0;
   map_or_bot_ = 0;
   quatX_ = 0;
   quatY_ = 0;
   quatZ_ = 0;
   quatW_ = 0;
   command_ = "";
   cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   // subscribe to keyboard and motion
    motionCommandsSub_ = nh_.subscribe("motion_cmd", 5, &movementControl::motionCommandCallback, this);
   keyboardCommandsSub_ = nh.subscribe("keyboard", 1, &movementControl::keyboardCommandCallback, this);
   ac_ = new MoveBaseClient("move_base",true);
   //wait for the action server to come up
   while(!ac_->waitForServer(ros::Duration(5.0))){
   ROS_INFO("commands controller is waiting for the move_base action server to come up");
   }
   ROS_INFO("commands controller sees that the move_base action server is up");
}
   ~movementControl()
   {
      delete ac_;
   }

void sendMotionCommand(std::string motionCmd)
{  
   std_msgs::String msgSpeech;
   move_base_msgs::MoveBaseGoal goal;
   // some goal parameters never change (e.g., we cannot translate in z or rotate in x  or y)
   goal.target_pose.pose.position.z = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   //we will be sending commands of type "twist"
   geometry_msgs::Twist base_cmd;
   // our bot does not use these motions, set them to 0
   base_cmd.angular.x = 0.0;
   base_cmd.angular.y = 0.0;
   base_cmd.linear.y = 0.0;
   // we do use these motions, but start them at 0 too
   base_cmd.linear.x = 0.0;
   base_cmd.linear.z = 0.0;   // this is our enable command.  start enabled.
   base_cmd.angular.z = 0.0;   

   bool finishedInTime;

   //move forward
   if(motionCmd[0]=='f'){
     base_cmd.linear.x = 0.15;
     base_cmd.angular.z = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   } 
   //turn left (yaw) 
   else if(motionCmd[0]=='l'){
     base_cmd.angular.z = 0.75;
     base_cmd.linear.x = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   } 
   //turn right (yaw)
   else if(motionCmd[0]=='r'){
     base_cmd.angular.z = -0.75;
     base_cmd.linear.x = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //move backwards
   else if(motionCmd[0]=='b'){
     base_cmd.linear.x = -0.15;
     base_cmd.angular.z = 0.0;
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   } 
   //stop
   else if(motionCmd[0]=='s'){
     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = 1.0; // we cannot move vertically, this is our re-enable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //disable
  else if(motionCmd[0]=='x'){

     base_cmd.angular.z = 0.0;
     base_cmd.linear.x = 0.0;
     base_cmd.linear.z = -1.0; // we cannot move vertically, this is our disable command
     //publish the assembled command
     cmd_vel_pub_.publish(base_cmd);
   }
   //cancel goal
   else if(motionCmd[0]=='c'){

     ac_->cancelAllGoals();
     ROS_INFO("canceling all goals");
   }	
   // move 1 meter forward
   else if(motionCmd[0]=='m'){
     	//we'll send a goal to the robot to move 1 meter forward
     	goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 1.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.z = 0.0;

      ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Hooray, the base moved 1 meter forward");
          }
          else
          {
            ROS_INFO("The base failed to move forward 1 meter for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
      }
   }
   // move 10 meters forward 
   else if(motionCmd[0]=='n'){
     	//we'll send a goal to the robot to move 1 meter forward
     	goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 10.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.z = 0.0;

      ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
     
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Hooray, the base moved 1 meter forward");
          }
          else
          {
            ROS_INFO("The base failed to move forward 1 meter for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
      }

   }
   // turn 90 degrees to the right
   else if(motionCmd[0]=='t'){
      goal.target_pose.header.frame_id = "base_link";
     	goal.target_pose.header.stamp = ros::Time::now();

     	goal.target_pose.pose.position.x = 0.0;
      goal.target_pose.pose.position.y = 0.0;
     	goal.target_pose.pose.orientation.w = sqrt(0.5);
      goal.target_pose.pose.orientation.z = -sqrt(0.5);

     	ROS_INFO("Sending goal");
     	ac_->sendGoal(goal);
      finishedInTime = ac_->waitForResult(ros::Duration(15));
      if (finishedInTime)
      {
          if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Hooray, the base turned 90 degrees right");
          }
          else
          {
            ROS_INFO("The base failed to turn 90 degrees right for some reason"); 
          }
      }
      else
      {
         ROS_INFO("Timed out moving to goal");
      }
   }
   // go to a specified pose on the map
   else if(motionCmd[0]=='p' && motionCmd.length() > 10)
   {
      ROS_INFO("publishing pose command received");
      goal.target_pose.header.frame_id = "map";
     	goal.target_pose.header.stamp = ros::Time::now();

      // take off the leading "p"
      std::string poseCmd = motionCmd.substr(1,std::string::npos);
      // locate the first comma, which arrives after the "x" value
      std::string poseDataBuffer[32];
      std::size_t found = poseCmd.find_first_of(",");
      int numPoseValues = 0;
      while (found!=std::string::npos)
      {
         poseDataBuffer[numPoseValues] = poseCmd.substr(0,found);
         numPoseValues++;
         poseCmd = poseCmd.substr(found + 1, std::string::npos);
         found = poseCmd.find_first_of(",");
      }

      if (numPoseValues == 4)
      {
         goal.target_pose.pose.position.x = atof(poseDataBuffer[0].c_str());
         goal.target_pose.pose.position.y = atof(poseDataBuffer[1].c_str());
         goal.target_pose.pose.orientation.z = atof(poseDataBuffer[2].c_str());
         goal.target_pose.pose.orientation.w = atof(poseDataBuffer[3].c_str());

     	   ROS_INFO("Sending goal pose");
     	   ac_->sendGoal(goal);
      }
      else ROS_WARN("Goal pose request has incorrect number of parameters");
   }

}

void keyboardCommandCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string keyboardCommand = msg->data.c_str();
   sendMotionCommand(keyboardCommand);

}

void motionCommandCallback(const outdoor_bot::movement_msg::ConstPtr &msg)
{
   command_ = msg->command;
   map_or_bot_ = msg->map_or_bot;
   if (map_or_bot_ == 0) // move to a pose on the map
   {
      quatX_ = msg->quatX;
      quatY_ = msg->quatY;
      quatZ_ = msg->quatZ;
      quatW_ = msg->quatW;
   }
   else if (map_or_bot_ == 1) // move with respect to the current robot location
   {
      distance_ = msg->distance;
      angle_ = msg->angle;
   }
   sendMotionCommand(command_);
}
};

int main(int argc, char** argv)
{
   //init the ROS node
   ros::init(argc, argv, "commands_controller");
   ros::NodeHandle nh;
   movementControl mC(nh);

   ros::spin();

}


