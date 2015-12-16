#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Joy.h> 
#include "std_msgs/Bool.h"
#include <string>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <asctec_hl_comm/HL_interface.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <algorithm>
// Button definitions
// TRIGGER
const int BTN_TRIGGER = 0;
const int BTN_PROCEED = 8;
const int BTN_CANCEL = 6;
const int BTN_ESTOP = 11;
// Standby Mode
const int BTN_STANDBY_GO_OPERATIONAL = 2;
// Operational Mode
const int BTN_OPERATIONAL_MODE_ATTITUDE = 2;
const int BTN_OPERATIONAL_MODE_HOVER = 3;

bool motors_on;
bool get_joy;

double pitch_deadband; 	// joystick units, 0.1 = 10% of half-range
double roll_deadband; // joystick units, 0.1 = 10% of half-range
double yaw_deadband; // joystick units, 0.1 = 10% of half-range
double freq;

using namespace std;

class JoyControl
{

	
public:

  JoyControl()  
  {
	  motors_on = true;
          get_joy = false;
	  roll_deadband = 0.05;
	  pitch_deadband = 0.05;
	  nh.param("freq", freq, 20.0);
	  
	  joy_sub = nh.subscribe("/joy", 1, &JoyControl::joyCallBack, this);
	  cmd_pub =  nh.advertise<asctec_hl_comm::mav_ctrl> ("/control", 1);
	  publish_timer = nh.createTimer(ros::Duration(1/freq), &JoyControl::publishCmdCallBack, this);
  }
 private:
  ros::NodeHandle nh;

  ros::Subscriber joy_sub;
  
  ros::Publisher cmd_pub;

  ros::Timer publish_timer;

  sensor_msgs::Joy joy_msg;
  
  sensor_msgs::Joy recent_joy;
  
  asctec_hl_comm::mav_ctrl ctrl_msg;
  
  
  void joyCallBack(const sensor_msgs::JoyConstPtr& joyMsg)
   {
 	  ros::Time rightNow = ros::Time::now();
 	//   ROS_INFO("debug0");
 	  //unvalid message
 	  if(joyMsg->axes.size() == 0 )
 	  {

 		  ROS_INFO("invalid message!");
 		  return;
 	  }

 	  
	  recent_joy = *joyMsg;
          get_joy = true;
          

 
 	  	  
  }

   void publishCmdCallBack(const ros::TimerEvent& e){
	   
	  ROS_DEBUG_STREAM("Timer callback triggered " << e.current_real.toSec()); 
	  // ROS_INFO("debug1");
	
	
	// nh.setParam("/fcu/position_control","off");
      //   nh.setParam("/fcu/state_estimation","off");
	 if(motors_on && get_joy){
		 double roll_in = double(recent_joy.axes[0]);
		 double pitch_in = double(recent_joy.axes[1]);
		 double yaw_in = double(recent_joy.axes[2]);
		 //determine the sign
		 double roll_sign = (roll_in > 0) ? 1.0 : -1.0;
		 double pitch_sign = (pitch_in > 0) ? 1.0 : -1.0;
		 double yaw_sign = (yaw_in > 0) ? 1.0 : -1.0;
 	  
 	 ctrl_msg.type = asctec_hl_comm::mav_ctrl::acceleration;
 	  ctrl_msg.y= roll_sign * roll_in > roll_deadband ? (std::max(-1.0, std::min(1.0, roll_in)) - roll_sign
 	          * roll_deadband) / (1.0 - roll_deadband) : 0.0;
 	  ctrl_msg.x = pitch_sign * pitch_in > pitch_deadband ? -(max(-1.0, min(1.0, pitch_in)) - pitch_sign
 	          * pitch_deadband) / (1.0 - pitch_deadband) : 0.0;
 	  
 	  double alt_cmd = (min(1.0, max(-1.0, double(recent_joy.axes[3]))) + 1.0) / 2.0; // clip, offset, scale to 0..1 range
          ctrl_msg.z = alt_cmd; 
 	  if (alt_cmd > 0.05)
 	       {
 		      ctrl_msg.yaw = yaw_sign * yaw_in > yaw_deadband ? (max(-1.0, min(1.0, yaw_in)) - yaw_sign
 	             * yaw_deadband) / (1.0 - yaw_deadband) : 0.0;
 	       }
 	       else
 	       {
 	    	   ctrl_msg.yaw = 0; // don't issue yaw command if alt lever is bottomed out, user probably wants to turn on/off motors
 	       }
               
 	       ctrl_msg.v_max_xy = 1; // use max velocity from config
	       ctrl_msg.v_max_z = 1; 
	 }
	   else{
		  
		   ctrl_msg.x = 0;
		   ctrl_msg.y = 0;
		   ctrl_msg.z = -1;
		   ctrl_msg.yaw = 0;
		   ctrl_msg.v_max_xy = 1; // use max velocity from config
		   ctrl_msg.v_max_z = 1;
	   }
 	       
 	       cmd_pub.publish(ctrl_msg);	     
 	  
   }
  

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_control_test");
  JoyControl controller_test = JoyControl();
  ros::Rate loop_rate(30); 
  ros::spin();
  return 0;
}


