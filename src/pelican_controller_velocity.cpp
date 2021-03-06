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

static double EXP = 2.718281828459; //e
// Never output commands greater than these values no matter what parameters are set:
static double ROLL_SCALE = M_PI / 180.0; // rad/deg
static double PITCH_SCALE = M_PI / 180.0; // rad/deg
static double MAX_ROLL_CMD = M_PI_2; // rad
static double MAX_PITCH_CMD = M_PI_2; // rad
static double MAX_YAW_RATE_CMD = 2.0 * M_PI; // rad/s
static double MAX_THRUST_CMD = 32; // N

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
	  motors_on = false;
	  roll_deadband = 0.05;
	  pitch_deadband = 0.05;
	  nh.param("freq", freq, 20.0);
	  
	  joy_sub = nh.subscribe("/joy", 10, &JoyControl::joyCallBack, this);
	  cmd_pub =  nh.advertise<asctec_hl_comm::mav_ctrl> ("/fcu/control", 1);
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
 	 //ROS_INFO("debug0");
 	  //unvalid message
 	  if(joyMsg->axes.size() == 0 )
 	  {

 		  ROS_INFO("invalid message!");
 		  return;
 	  }
 	   // stop
 	  
	  recent_joy = *joyMsg;
	  
	  //ESTOP
 	  if(joyMsg->buttons[10] == 1){
 		  ROS_INFO("Stopping motors!");
 		 motors_on = false;
 		 asctec_hl_comm::mav_ctrl_motors::Request req;
 		 asctec_hl_comm::mav_ctrl_motors::Response res;
 		 req.startMotors = 0;
 		 ros::service::call("fcu/motor_control", req, res);
 		  return;
 	  }
 
 	  //turn on the motors
 	  if (not motors_on and //
 	              (joyMsg->buttons[1]) and (joyMsg->axes[3] <= -0.95) // full right yaw and full min altitude
 	          )
 	          {
 	            
 	           nh.setParam("/fcu/position_control","off");
 	           asctec_hl_comm::mav_ctrl_motors::Request req;
 	           asctec_hl_comm::mav_ctrl_motors::Response res;
 	           req.startMotors = 1;
 	           ros::service::call("fcu/motor_control", req, res);
 	           std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
                   if(res.motorsRunning){
                       motors_on = true;
 	               ROS_INFO("motors on");

                    }
 	          //  NODELET_INFO("Motors ON");
 	          }

 	  //turn off the motors
 	  else if (motors_on and //
 	              (joyMsg->buttons[3]) and (joyMsg->axes[3] <= -0.95) // full left yaw and full min altitude
 	          )
 	          {
 	           nh.setParam("/fcu/position_control","off");
 	           asctec_hl_comm::mav_ctrl_motors::Request req;
 	           asctec_hl_comm::mav_ctrl_motors::Response res;
 	           req.startMotors = 0;
 	           ros::service::call("fcu/motor_control", req, res);
 	           std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
                   if(!res.motorsRunning){
                       motors_on = false;
 	               ROS_INFO("motors off");

                    }
 	          }
 
 	  	  
  }

   void publishCmdCallBack(const ros::TimerEvent& e){
	   
	  ROS_DEBUG_STREAM("Timer callback triggered " << e.current_real.toSec()); 
	  // ROS_INFO("debug1");
	
	
	 nh.setParam("/fcu/position_control","off");
         nh.setParam("/fcu/state_estimation","off");
	 if(motors_on){
		 double roll_in = double(recent_joy.axes[0]);
		 double pitch_in = double(-recent_joy.axes[1]);
		 double yaw_in = double(recent_joy.axes[2]);
		 //determine the sign
		 double roll_sign = (roll_in > 0) ? 1.0 : -1.0;
		 double pitch_sign = (pitch_in > 0) ? 1.0 : -1.0;
		 double yaw_sign = (yaw_in > 0) ? 1.0 : -1.0;
 	  
		 //take off experimental
		 if(recent_joy.buttons[0]){
			 nh.setParam("/fcu/position_control","Highlevel");
                         nh.setParam("/fcu/state_estimation","Extern");

			 ctrl_msg.type = asctec_hl_comm::mav_ctrl::position;
			 ctrl_msg.x = 0;
			 ctrl_msg.y = 0;
			 ctrl_msg.z = 0.5;
			 ctrl_msg.yaw = 0;
			 ctrl_msg.v_max_xy = 5.0; // use max velocity from config
			 ctrl_msg.v_max_z = 5;
                         ROS_INFO("I'm here");
 	  }
		 else{
 	  ctrl_msg.type = asctec_hl_comm::mav_ctrl::acceleration;

 	  ctrl_msg.y= roll_sign * roll_in > roll_deadband ? (std::max(-1.0, std::min(1.0, tan(M_PI*roll_in/4.0))) - roll_sign
 	          * roll_deadband) / (1.0 - roll_deadband) : 0.0;
 	  ctrl_msg.x = pitch_sign * pitch_in > pitch_deadband ? (max(-1.0, min(1.0, tan(M_PI*pitch_in/4.0))) - pitch_sign
 	          * pitch_deadband) / (1.0 - pitch_deadband) : 0.0;
 	  double alt_cmd = (min(1.0, max(-1.0, double(recent_joy.axes[3]))) + 1.0) / 2.0; // clip, offset, scale to 0..1 range
          ctrl_msg.z = alt_cmd; 
 	  if (alt_cmd > 0.05)
 	       {
 		      ctrl_msg.yaw = yaw_sign * yaw_in > yaw_deadband ? (max(-1.0, min(1.0, tan(M_PI*yaw_in/4.0))) - yaw_sign
 	             * yaw_deadband) / (1.0 - yaw_deadband) : 0.0;
 	       }
 	       else
 	       {
 	    	   ctrl_msg.yaw = 0; // don't issue yaw command if alt lever is bottomed out, user probably wants to turn on/off motors
 	       }
               
 	       ctrl_msg.v_max_xy = 1.0; // use max velocity from config
	       ctrl_msg.v_max_z = 5.0;
	   } 
	 }
	   else{
		   ctrl_msg.type = asctec_hl_comm::mav_ctrl::acceleration;
		   ctrl_msg.x = 0;
		   ctrl_msg.y = 0;
		   ctrl_msg.z = 0;
		   ctrl_msg.yaw = 0;
		   ctrl_msg.v_max_xy = 1.0; // use max velocity from config
		   ctrl_msg.v_max_z = 5.0;
	   }
 	       
 	       cmd_pub.publish(ctrl_msg);	     
 	  
   }
  

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_control");
  JoyControl controller = JoyControl();
  ros::Rate loop_rate(30); 
  ros::spin();
  return 0;
}


