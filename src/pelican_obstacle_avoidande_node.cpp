#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/mav_ctrl_motors.h>
#include <math.h>
#include <altitude_sensor/sensor_data.h>
using namespace std;

class PelicanAvoidanceClass
{
private:
	ros::NodeHandle nh;
	//lidar information
	ros::Subscriber point_sub;
	//joy information
	ros::Subscriber joy_sub;
    //position_information
     ros::Subscriber position_sub; 
	//command publisher
	ros::Publisher cmd_pub;
	ros::Timer publisher_timer;
        
	double deadband; 	// joystick units, 0.1 = 10% of half-range
	
	sensor_msgs::Joy joy_msg;
	sensor_msgs::Joy recent_joy;

	//command message
	asctec_hl_comm::mav_ctrl ctrl_msg_out;
	asctec_hl_comm::mav_ctrl ctrl_msg_in;

	bool get_joy;
	bool motors_on;
	//max velocity 
	double v_max;
	
	//security distance
	double epsi;
	//decide if need filtering
	bool need_filter;
	bool filtered_vel;
    double al_xy;
	double filtered_v_x;
	double filtered_v_y;
	double d_max;

	//take off and land mode
	bool take_off_mode;
	bool land_mode;
	bool took_off;
	double recent_z;

public:
	PelicanAvoidanceClass()
	{

		joy_sub =  nh.subscribe("/joy", 10, &PelicanAvoidanceClass::joyCallback,this);
        	position_sub =  nh.subscribe("/altitude", 2, &PelicanAvoidanceClass::positionCallback,this);
		cmd_pub = nh.advertise<asctec_hl_comm::mav_ctrl>("/fcu/control",1);
		point_sub = nh.subscribe("/lidar_points_dangerous",2, &PelicanAvoidanceClass::pointCallback,this);
		publisher_timer = nh.createTimer(ros::Duration(1/50.0), &PelicanAvoidanceClass::publishCmdCallBack, this);
		deadband = 0.05;
       		epsi = 0.8;
		get_joy = false;
		motors_on = false;
		need_filter = false;
		take_off_mode = false;
		land_mode = false;
		took_off = false;

		v_max = 0.2;
		nh.param("al_xy",al_xy,0.35);
		nh.param<double>("d_max", d_max, 0.8);
	}


private:
	void joyCallback(const sensor_msgs::JoyConstPtr& joyMsg)
	{
	    ros::Time rightNow = ros::Time::now();
               
	      

 	  	//unvalid message        
 	  	if(joyMsg->axes.size() == 0){
                         ROS_INFO("debug0");
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
 			ROS_INFO("invalid message!");
			return;
 	  	}
 	 
	  	recent_joy = *joyMsg;
		get_joy = true;
	         ROS_INFO("debug1");
	  	//ESTOP with button 11
 	  	if(joyMsg->buttons[10] == 1){	
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
			asctec_hl_comm::mav_ctrl_motors::Request req;
			asctec_hl_comm::mav_ctrl_motors::Response res;
			req.startMotors = 0;
			ros::service::call("fcu/motor_control", req, res);
            		if(!res.motorsRunning){
				ROS_INFO("Stopping motors in emergency!");
			}
			motors_on = false; 
			return;
		}
 
		//turn on the motors with button 2
		if (not motors_on and joyMsg->buttons[1]){ 
                         ROS_INFO("debug2");
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
 	        	asctec_hl_comm::mav_ctrl_motors::Request req;
 	       		asctec_hl_comm::mav_ctrl_motors::Response res;
 	        	req.startMotors = 1;
 	        	ros::service::call("fcu/motor_control", req, res);
 	        	//std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
            		if(res.motorsRunning){
				motors_on = true;
				ROS_INFO("motors on");
			}

        	}
		else if(motors_on){
			//take off  with button 1
			if(not took_off and joyMsg->buttons[0]){
                                
				if(joyMsg->axes[3] != 0){
                                        ROS_INFO("debug3");
                                        nh.setParam("/fcu/position_control","off");
					nh.setParam("/fcu/fcu/position_control","off");
					take_off_mode = false;
					ROS_ERROR("The altitude command is not put at 0, please fix it.");
				}
				else{
                                        ROS_INFO("debug4");
                                        nh.setParam("/fcu/position_control","HighLevel");
					nh.setParam("/fcu/fcu/position_control","HighLevel");
					take_off_mode = true;
				}
			}

			//land mode 
			else if(took_off and joyMsg->buttons[0]){
                                nh.setParam("/fcu/position_control","HighLevel");
				nh.setParam("/fcu/fcu/position_control","HighLevel");
				land_mode = true;
			}

			//turn off the motors with button 4
			else if ( (joyMsg->buttons[3]))
 	          {
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
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
		else{
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
		}
	}

	//get altitude information
     void positionCallback(altitude_sensor::sensor_data msg){
		 recent_z = msg.altitude;
                 //ROS_INFO("%f", recent_z);
        }


	void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		pcl::PointCloud<pcl::PointXYZ> temp;
		pcl::fromROSMsg(*msg, temp);
		unsigned int n_pcl = temp.size();
		double min_dis = 10.0;
		unsigned int  mark_of_min;
		//find the most dangerous one
		if(n_pcl > 0){
			for (unsigned int i = 0; i <n_pcl;i++){
				double x, y ,z;
				x= temp[i].x;
				y = temp[i].y;
				z= temp[i].z;
				double dis = sqrt(x*x + y*y) ;
				if(dis < min_dis){
					min_dis = dis;
					mark_of_min = i;
				}
			}
		}
		need_filter = false;
		if( min_dis >0.001 && min_dis < d_max){
			need_filter = true;
		}
		if(need_filter){
			double norm =  sqrt(temp[mark_of_min].x*temp[mark_of_min].x + temp[mark_of_min].y*temp[mark_of_min].y);
			filtered_v_x =  - temp[mark_of_min].x/norm*(1.0*cos(M_PI*(min_dis - 0.325)/(2*d_max)));
			filtered_v_y =  - temp[mark_of_min].y/norm*(1.0*cos(M_PI*(min_dis - 0.325)/(2*d_max)));
		}

	}
	
	void publishCmdCallBack(const ros::TimerEvent& e){
		//ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::acceleration;
		double v_tx, v_ty, v_tz, v_tyaw;
                
		if(motors_on){
			//take off mode
			if(take_off_mode and not took_off){
                                 ROS_INFO("debug5");
				nh.setParam("/fcu/position_control","HighLevel");
				nh.setParam("/fcu/fcu/position_control","HighLevel");
				nh.setParam("/fcu/fcu/state_estimation", "HighLevel_SSDK");
				ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::velocity;
				if(recent_z < 0.40){
					v_tx = 0.0;
					v_ty = 0.0;
					v_tz = 0.1*cos(M_PI*recent_z) + 0.1;
					v_tyaw = 0.0;
				}
				else if(recent_z > 0.50){
					v_tx = 0.0;
					v_ty = 0.0;
					v_tz = 0.1*cos(M_PI*recent_z)- 0.1;;
					v_tyaw = 0.0;
				}
				else{
					v_tx = 0.0;
					v_ty = 0.0;
					v_tz = 0.0;
					v_tyaw = 0.0;
					take_off_mode = false;
					took_off = true;
					ROS_INFO("Succesfully taken off. Entering manual mode.");
				}
			}
			//manual mode
			else if(took_off){
                                ROS_INFO("debug6");
				if(land_mode){					
					nh.setParam("/fcu/fcu/position_control","HighLevel");
					nh.setParam("/fcu/fcu/state_estimation", "HighLevel_SSDK");
					ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::velocity;
					if(fabs(recent_z - 0.05) > 0.0005){
						v_tx = 0.0;
						v_ty = 0.0;
						v_tz = - 0.2;
						v_tyaw = 0.0;
					}
					else{
                        			nh.setParam("/fcu/position_control","off");
    						nh.setParam("/fcu/fcu/position_control","off");
                                                ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::acceleration;
						asctec_hl_comm::mav_ctrl_motors::Request req;
						asctec_hl_comm::mav_ctrl_motors::Response res;
						req.startMotors = 0;
						ros::service::call("fcu/motor_control", req, res);
						std::cout << "motors running: " << (int)res.motorsRunning << std::endl;
						if(!res.motorsRunning){
							motors_on = false;
							ROS_INFO("Land finished.");
						}
					}
				}
				else{
					nh.setParam("/fcu/fcu/position_control","HighLevel");
					nh.setParam("/fcu/fcu/state_estimation", "HighLevel_SSDK");
					ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::velocity;
					v_tx = recent_joy.axes[1];
					v_ty = recent_joy.axes[0];
					v_tz = recent_joy.axes[3];
					v_tyaw = recent_joy.axes[2];
					double x_sign = (v_tx > 0) ? 1.0 : -1.0;
					double y_sign = (v_ty > 0) ? 1.0 : -1.0;
					double z_sign = (v_tz > 0) ? 1.0 : -1.0;
					double yaw_sign = (v_tyaw > 0) ? 1.0 : -1.0;
					//ctrl_msg_in.type = asctec_hl_comm::mav_ctrl::acceleration;
					v_tx = x_sign * v_tx > deadband ? -(max(-1.0, min(1.0, v_tx)) - x_sign*deadband) / (1.0 - deadband)/al_xy : 0.0;
					v_ty = y_sign * v_ty > deadband ? (max(-1.0, min(1.0, v_ty)) - y_sign*deadband) / (1.0 - deadband)/al_xy : 0.0;
					v_tz = z_sign * v_tz > deadband ? (max(-1.0, min(1.0, v_tz)) - z_sign*deadband) / (1.0 - deadband) : 0.0;
					v_tyaw = yaw_sign * v_tyaw > deadband ? (max(-1.0, min(1.0, v_tyaw)) - yaw_sign*deadband) / (1.0 - deadband) : 0.0;
					
					//double alt_cmd = (min(1.0, max(-1.0, double(recent_joy.axes[3]))) + 1.0) / 2.0; // clip, offset, scale to 0..1 range
					/* if (alt_cmd > 0.05){
						ctrl_msg_in.yaw = yaw_sign * yaw_in > yaw_deadband ? (max(-1.0, min(1.0, yaw_in)) - yaw_sign* yaw_deadband) / (1.0 - yaw_deadband) : 0.0;
					 }
					else{
						ctrl_msg_in.yaw = 0; // don't issue yaw command if alt lever is bottomed out, user probably wants to turn on/off motors
					}  */
					if(need_filter){
						v_tx = filtered_v_x;
						v_ty = filtered_v_y;
						filtered_vel = true;
					}
					else{
						filtered_vel = false;
					}
				}
			}
                  else{
                                         ROS_INFO("debug7");
			
					nh.setParam("/fcu/position_control","off");
					nh.setParam("/fcu/fcu/position_control","off");
					ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::acceleration;
					v_tx = 0.0;
					v_ty = 0.0;
					v_tz = 0.0;
					v_tyaw = 0.0;
				}
                       

		}
		else{
                         ROS_INFO("debug8");
			
			nh.setParam("/fcu/position_control","off");
			nh.setParam("/fcu/fcu/position_control","off");
			ctrl_msg_out.type = asctec_hl_comm::mav_ctrl::acceleration;
			v_tx = 0.0;
			v_ty = 0.0;
			v_tz = 0.0;
			v_tyaw = 0.0;
		}

		ctrl_msg_out.x = v_tx;
		ctrl_msg_out.y = v_ty;
		ctrl_msg_out.z = v_tz;
		ctrl_msg_out.yaw = v_tyaw;
		ctrl_msg_out.v_max_xy = v_max;
		ctrl_msg_out.v_max_z = 2.5;
		cmd_pub.publish(ctrl_msg_out);
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "joy_control");
	PelicanAvoidanceClass pelicanAvoidance = PelicanAvoidanceClass();
	ros::spin();
	return 0;
}

