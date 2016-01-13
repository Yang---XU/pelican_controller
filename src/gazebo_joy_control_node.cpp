#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Joy.h> 
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
using namespace std;

class JoyControlClass{

public:
	ros::NodeHandle nh;
	ros::Subscriber joy_sub;
	ros::Subscriber position_sub;
	ros::Publisher cmd_pub;
        ros::Timer publish_timer;
	sensor_msgs::Joy joy_msg;
        sensor_msgs::Joy recent_joy;
	geometry_msgs::Twist velocity;
        bool take_off_mode ;
        bool land_mode;
	bool first_joy_received;
	bool motors_on;
	bool took_off;
	double v_x_max;
	double v_y_max;
	double v_z_max;
	double v_yaw_max;
	double recent_z;
	double deadband;
  JoyControlClass()
  {
	  
	  position_sub = nh.subscribe("gazebo/model_states", 1, &JoyControlClass::positionCallback, this);
          joy_sub = nh.subscribe("joy", 1, &JoyControlClass::joyCallback, this);
	  cmd_pub =  nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
          publish_timer = nh.createTimer(ros::Duration(1/20.0), &JoyControlClass::publishCmdCallBack, this);
	  first_joy_received = false;
          take_off_mode = false;
          land_mode = false;
	  took_off = false;
	  motors_on = false;
	  nh.param<double>("v_x_max", v_x_max, 2.0);
	  nh.param<double>("v_y_max", v_y_max, 2.0);
	  nh.param<double>("v_z_max", v_z_max, 2.0);
	  nh.param<double>("v_yaw_max", v_yaw_max, M_PI/2.0);
	  nh.param<double>("deadband", deadband, 0.005);

  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
          recent_joy = *joy;
	  //start the motors
	  if(not motors_on and joy->buttons[1]){
		  motors_on = true;
		  ROS_INFO("motors running");
	  }
	  if(motors_on){
		  
		  //take off with button 1
		  if(not took_off and joy->buttons[0]){
                     if(joy->axes[3] == 0){
                         take_off_mode = true;
                    }
                     else{
                         ROS_ERROR("The attitude commande is not put at 0. Please fix it."); 
                    }
                    }
                   //land 
                  if(took_off and joy->buttons[0]){
                      land_mode = true;                      
		  }
         
               }   
		  //during the flight
		  
  }


  void positionCallback(const gazebo_msgs::ModelStates::ConstPtr& sta){
	  gazebo_msgs::ModelStates state = *sta;
	  recent_z = state.pose[1].position.z;
         
  }
 
 void publishCmdCallBack(const ros::TimerEvent& e){
     if(motors_on){
     if(take_off_mode and not took_off){
                        std::cout << "current height: " << (double)recent_z << std::endl; 
		  	if(recent_z < 0.45){
			  velocity.linear.x = 0.0;
			  velocity.linear.y = 0.0;
			  velocity.linear.z = 0.1*cos(M_PI*recent_z) + 0.1;
			  velocity.angular.z = 0.0;
		 	 }
			else if(recent_z > 0.52){
			  velocity.linear.x = 0.0;
			  velocity.linear.y = 0.0;
			  velocity.linear.z = 0.1*cos(M_PI*recent_z)- 0.1;;
			  velocity.angular.z = 0.0;
			}
		  	else{
			  velocity.linear.x = 0.0;
			  velocity.linear.y = 0.0;
			  velocity.linear.z = 0.0;
			  velocity.angular.z = 0.0;
                          take_off_mode = false; 
			  took_off = true;
			  }
     }
     if(took_off){
                        if(land_mode){
                           std::cout << "current height: " << (double)recent_z << std::endl; 
                           if(fabs(recent_z - 0.0848705) > 0.0005){
                          	 velocity.linear.x = 0.0;
			 	 velocity.linear.y = 0.0;
			 	 velocity.linear.z = -0.2;
			 	 velocity.angular.z = 0.0;                           
                             }
                           else{
                              ROS_INFO("land finished");
                              velocity.linear.x = 0.0;
			      velocity.linear.y = 0.0;
			      velocity.linear.z = -0.02;
			      velocity.angular.z = 0.0; 
                              took_off = false;
                              motors_on = false;
                              land_mode = false;
                            }  
			}  
                        else{
                        ROS_INFO("manual mode");
			double v_x = recent_joy.axes[1];
		    	double v_y = recent_joy.axes[0];
		  	double v_z = recent_joy.axes[3];
		  	double v_yaw = recent_joy.axes[2];

		 	 //determine the sign
		  	double x_sign = (v_x > 0) ? 1.0 : -1.0;	
		  	double y_sign = (v_y > 0) ? 1.0 : -1.0;
		  	double z_sign = (v_z > 0) ? 1.0 : -1.0;
 		  	double yaw_sign = (v_yaw > 0) ? 1.0 : -1.0;

		 	 v_x= x_sign*v_x > deadband ? (std::max(-1.0, min(1.0, v_x)) - x_sign*deadband)/(1.0 - deadband)*v_x_max:0.0;
		 	 v_y= y_sign*v_y > deadband ? (std::max(-1.0, min(1.0, v_y)) - y_sign*deadband)/(1.0 - deadband)*v_y_max:0.0;
		  	v_z= z_sign*v_z > deadband ? (std::max(-1.0, min(1.0, v_z)) - z_sign*deadband)/(1.0 - deadband)*v_z_max:0.0;
		 	 v_yaw= yaw_sign*v_yaw > deadband ? (std::max(-1.0, min(1.0, v_yaw)) - yaw_sign*deadband)/(1.0 - deadband)*v_yaw_max:0.0;
			  velocity.linear.x = v_x;
			  velocity.linear.y = v_y;
			  velocity.linear.z = v_z;
			  velocity.angular.z = v_yaw;
		    }
                 }		  		
	  }
     else{
        velocity.linear.x = 0.0;
	velocity.linear.y = 0.0;
	velocity.linear.z = 0.0;
	velocity.angular.z = 0.0;
     }
     cmd_pub.publish(velocity);

}




};



int main(int argc, char** argv){
	ros::init(argc, argv, "gazebo_joy_control");
	JoyControlClass controller = JoyControlClass();
        ROS_INFO("I m here");
	ros::Rate loop_rate(30);
	ros::spin();
	return 0;





}
