// myBot_node receives cmd_vel messages and publishes to 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <string>


//#define WHEEL_SEPARATION 0.36 // meters



ros::Publisher ucCommand;
ros::Subscriber moveCmd, ucResponse;

//bool roger_ = false; // use a launch file parameter to determine if this stays false or gets set to true
		// if it is true, it implies that we are connecting to a telepresence bot with a server, false for standard bot
		// all this does is change the command format to match what the arduino is expecting

void calculateMove(double vx, double vTheta)
{
   char intStr[33];	// needs to be large enough to take biggest possible number and /0
   std::string commandString;
   //if (roger_)	// command format for autobot, just send linear and ang vels
  // {
      sprintf(intStr,"%f",vx * 1000.);	// send instruction in mm/sec
      std::string value(intStr);
      commandString = "autoC(";
      commandString.append(intStr);
      commandString.append(",");
      sprintf(intStr,"%f",vTheta  * 57.1); // send instruction in degrees/sec
      commandString.append(intStr);
      commandString.append(");");
   //}
/*
   else	// command format for standard bot
   {
	double turning = vTheta * WHEEL_SEPARATION / 2; // gives the required wheel speed in m/s
			// for example to turn at 1 rotation per second, vTheta would = 2 * pi
			// and the distance the wheel travels is 2 * pi * r
			// because the circumference = 2 pi r
	//ROS_INFO("turning %f", turning);
	double moving = vx;  // we are only able to move forward, not to the side, so we cannot use vy
	//ROS_INFO("moving %f", moving);
	int rightWheelSpeed = (int) ((moving + turning) * 1000.);  // use mm/sec instead of m/s, so we can get integers
	int leftWheelSpeed = (int) ((moving - turning) * 1000.); 
	//ROS_INFO("left wheel speed %d", leftWheelSpeed);
	//ROS_INFO("right wheel speed %d", rightWheelSpeed);
	char intStr[33];	// needs to be large enough to take biggest possible number and /0
	sprintf(intStr,"%d",rightWheelSpeed);
	std::string rightWheelSpeedString(intStr);
	sprintf(intStr,"%d",leftWheelSpeed);
	std::string leftWheelSpeedString(intStr);

	commandString = "M";
	commandString.append(leftWheelSpeedString);
	commandString.append(",");
	commandString.append(rightWheelSpeedString);
	commandString.append("#");
   }   
*/  
   std_msgs::String msg;
   msg.data = commandString;
   ucCommand.publish(msg);	// send out a message to serial comm for transmission to the arduino
   //ROS_INFO("velocity command sent = %s", commandString.c_str());
}

void moveCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  
  // (vx, vy, vtheta) <==> (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z
  static bool enabled = true, disable_commanded = false;
  double vx = cmd_vel->linear.x;
  //double vy = cmd_vel->linear.y;  // we are not a holonomic robot, so vy is always 0
  double disable_indicator = cmd_vel->linear.z;
  double vTheta = cmd_vel->angular.z;
  //ROS_INFO("command received %f", vx);
  if (disable_indicator > 0.5)
  {
    ROS_INFO("commands re-enabled");
    enabled = true;
  }
  else if (disable_indicator < -0.5)
  {
    ROS_INFO("commands disabled");
    enabled = false;
    // when the disable command is issued, we want to send a single M0,0# command to get the bot to stop
    disable_commanded = true;	
  }
  if (enabled) calculateMove(vx, vTheta);
  else if (disable_commanded) // when the disable command is issued, we want to send a single M0,0# command to get the bot to stop
  {
    calculateMove(0.0,0.0);
    disable_commanded = false;
  }
}

int main(int argc, char** argv){
  //Initialize ROS
  ros::init(argc, argv, "baseController_node");
  ros::NodeHandle n;
  ROS_INFO("baseController_node starting");
  //if (argc > 1) roger_ = true; // use autobot formatting for output commands
  //moveCmd = n.subscribe("/base_controller/command", 50, moveCommandCallback);  // subscribe to move commands
  moveCmd = n.subscribe("cmd_vel", 50, moveCommandCallback);  // subscribe to move commands
  ucCommand = n.advertise<std_msgs::String>("uc1Command", 50); // advertise commands to be sent to the arduino's serial port
  while(n.ok()) ros::spinOnce();  // check for incoming messages
}
