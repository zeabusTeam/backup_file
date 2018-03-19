#include <ros/ros.h> //line 236 print all
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
//#include <zeabus_controller/control_force.h>
#include <zeabus_controller/point_xy.h>
#include <zeabus_controller/orientation.h>
#include <stdlib.h>
#include "PID_2018.cpp"
#include "manage_file.cpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <zeabus_controller/forcePIDConstantConfig.h>
#include <queue>
#include <iostream>
#include <zeabus_controller/drive_x.h>
#include <modbus_ascii_ros/Switch.h>

//this file don't write class type because this file is main run
#define PI 3.14159265
#define earth_gravity 9.780327
#define weight 40.2
#define mass 35.6
#define error_z 0.05
#define error_y 0.02
#define error_x 0.02
#define error_roll 0.1
#define error_pitch 0.1
#define error_yaw 0.035
#define error_axis 0.00
#define error_distance 0.0
#define check_correct(a,b,c) abs(a-b)<c?true:false
//double force_output[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//force about x y z roll pitch yaw
//----------------------------------------
static std::string tune_file = "force_SAUV.yaml";
//----------------------------------------
std::string force_message = "";
double force_output[6] = { 0 , 0 , 0 , 0 , 0 , 0};
double force_fix[6] = { 0 , 0 , 0 , 0 , 0 , 0 };
void calculate_out();
double distance_xy= 0 ;
double distance_yaw= 0 ;
double diff_yaw = 0 ;
PID *PID_position, *PID_velocity;
manage_PID_file PID_file(tune_file);

double Kp_position[6] = {0, 0, 0, 0, 0, 0};
double Ki_position[6] = {0, 0, 0, 0, 0, 0};
double Kd_position[6] = {0, 0, 0, 0, 0, 0};
// value of  acceeration from old code
double Kp_velocity[6] = {0, 0.01, 0.5, 0.2, 0.6, 0.0};
double Ki_velocity[6] = {0, 0, 0.0015, 0, 0, 0.005};
double Kd_velocity[6] = {0, 0, 0.0001, 0, 0, 0.001};
double Kvs_position[6] = {0, 0, 0, 0, 0, 0};

geometry_msgs::Twist message_cmd_vel;
geometry_msgs::Pose fix_position;
nav_msgs::Odometry current_state;
//double quaternion[4] = {0,0,0,0};

// setup tuning z
int step_work = 8; // 1 go depth : 2 tune z : 3 tune yaw : 4 tune pitch : 5 tune roll : 6 tune x : 7 tune y : 8 normal 
double target_depth = -1.0;

// x y z roll pitch yaw
double output[6] = {0, 0, 0, 0, 0, 0};
double current_velocity[6] = {0, 0, 0, 0, 0, 0}, target_velocity[6] = {0, 0, 0, 0, 0, 0};
double current_position[6] = {0, 0, 0, 0, 0, 0} , target_position[6] = {0, 0, target_depth, 0, 0, 0};
bool want_fix[6] = {false , false , false , false , false , false};
double error_position[6] = {0, 0, 0, 0, 0, 0} , error_velocity[6] = {0, 0, 0, 0, 0, 0}, normal_error[6] = {0, 0, 0, 0, 0, 0};
//double current_pose[7] = {0, 0, 0, 0, 0, 0, 0}, target_pose[7] = {0, 0, 0, 0, 0, 0, 0};

//zeabus_controller::control_force output_force_message;
bool start_run = true;
bool first_time_PID = true;
bool change_PID = false;
bool reset_position = false;

//about time
ros::Time previous_time;
ros::Time current_time;
double diff_time;


double ok_time = 1;

// setup tuning yaw
// bool unknow_torque_yaw = true;
// double previous_omega = 0.0;
// constant is torque/alpha
/*
bool tune_yaw = false;
bool tune_roll = false;
bool tune_pitch = false;
double constant_yaw = 0.0;
double constant_roll = 0.0;
double constant_pitch = 0.0;
*/
// setup function of ros
void listen_current_state(const nav_msgs::Odometry message);
void listen_target_velocity(const geometry_msgs::Twist message);
void listen_target_position(const geometry_msgs::Point message);
void listen_mode(const std_msgs::Int16 message);
void listen_absolute_xy(const zeabus_controller::point_xy message);
void listen_target_depth(const std_msgs::Float64 message);
void listen_absolute_yaw(const std_msgs::Float64 message);
void listen_absolute_orientation(const zeabus_controller::orientation message);
void listen_real_yaw(const std_msgs::Float64 message);
void config_constant_PID(zeabus_controller::forcePIDConstantConfig &config, uint32_t level);
std_msgs::Bool is_at_fix_position(double error);
std_msgs::Bool is_at_fix_orientation(double error);

//declare normal function
double get_roll_radian(double quaternion[4]);
double get_yaw_radian(double quaternion[4]);
double get_pitch_radian(double quaternion[4]);
double check_radian_tan(double result);
void message_to_quaternion(const geometry_msgs::Quaternion message , double* quaternion); 
double find_min_angular(double current, double target);
geometry_msgs::Twist twist_to_message(double twist[6]);
void set_all_PID();
void reset_all_I();
double abs(double a);
int abs(int a);
int found_sign(double direction);
void shutdown_force();
void shutdown_target_velocity();
bool same_direction(double num_01, double num_02);

//declare for testing
void test_current_state(const geometry_msgs::Point message);
void test_current_orientation(const zeabus_controller::orientation message);

//declare calculate variable
int fast_velocity_z = 0;
int fast_velocity_x = 0;
int fast_velocity_y = 0;
double absolute_error = 0;
double right_yaw = 0;
double left_yaw = 0;
bool shutdown_force_z = false; 
bool shutdown_force_xy = false;

// print datail from ros log
bool print_fatal = true;
// initial code run 1 time
void init(){
	PID_velocity = (PID*)calloc(6, sizeof(PID));
	PID_position = (PID*)calloc(6, sizeof(PID));
	for(int count = 0 ; count < 6 ; count++){
		PID_position[count].set_PID(Kp_position[count], Ki_position[count], Kd_position[count], Kvs_position[count]);
		PID_velocity[count].set_PID(Kp_velocity[count], Ki_velocity[count], Kd_velocity[count], 0);
		PID_position[count].reset_I();
		PID_velocity[count].reset_I();
	}
}

// main file
int main(int argc, char **argv){
// setup ros system
	ros::init(argc, argv, "force_controller");
	ros::NodeHandle nh;
// setup test topic
	ros::Subscriber test_state = nh.subscribe("/test/point" , 1000, &test_current_state);
	ros::Subscriber test_orientation = nh.subscribe("/test/orientation", 1000, &test_current_orientation);
// setup listen topic
	ros::Subscriber sub_state = nh.subscribe("/auv/state" , 1000, &listen_current_state);
	ros::Subscriber sub_target_velocity = nh.subscribe("/zeabus/cmd_vel", 1, &listen_target_velocity);
	ros::Subscriber sub_target_position = nh.subscribe("/cmd_fix_position", 1000, &listen_target_position);
	ros::Subscriber sub_controller = nh.subscribe("/zeabus_controller/mode", 1000, &listen_mode);
	ros::Subscriber sub_target_depth = nh.subscribe("/fix/abs/depth", 1000, &listen_target_depth);
	ros::Subscriber sub_absolute_yaw = nh.subscribe("/fix/abs/yaw", 1000, &listen_absolute_yaw);
	ros::Subscriber sub_real_yaw = nh.subscribe("/fix/rel/yaw", 1000, &listen_real_yaw);
	ros::Subscriber sub_absolute_xy = nh.subscribe("fix/abs/xy", 1000, &listen_absolute_xy);
	ros::Subscriber sub_absolute_orientation = nh.subscribe("/fix/abs/orientation", 1000, &listen_absolute_orientation);
// setup service topic
//	ros::ServiceServer sub_target_x_y = nh.advertiseService("/fix/rel/x", 1000, &listen_target_x);
	
// setup tell topic
//	ros::Publisher tell_pub = nh.advertise<zeabus_controller::control_force>("/control/force", 1000)
	ros::Publisher tell_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher is_at_fix_position_pub = nh.advertise<std_msgs::Bool>("/zeabbus_controller/is_at_fix_position",1000);
	ros::Publisher is_at_fix_orientation_pub = nh.advertise<std_msgs::Bool>("zeabus_controller/is_at_fix_orientation",1000);
	
// setup dynamic configK of PID
	dynamic_reconfigure::Server<zeabus_controller::forcePIDConstantConfig> server;
	dynamic_reconfigure::Server<zeabus_controller::forcePIDConstantConfig>::CallbackType f;

	init();
	std::cout << "mass of auv is " << mass << std::endl;
// 100% copy
	f = boost::bind(&config_constant_PID, _1, _2);
	server.setCallback(f);
	ros::Rate rate(50);
	ros::Rate sleep(10);
	while(nh.ok()){
		if(first_time_PID){
			std::cout << "Before download" << std::endl; 
			PID_file.load_file("Controller");
			std::cout << "Finish download" << std::endl;
			first_time_PID = false;
			set_all_PID();
 			sleep.sleep();
		}
		else if(change_PID){
			std::cout << "before save file" << std::endl;
			PID_file.save_file("Controller");
			std::cout << "finish save file" << std::endl;
			change_PID = false;
		}
		else{}
		ros::spinOnce();
		if(step_work == 1){
		}
		else if(step_work==8){
			for(int i = 0; i < 6 ; i++){
		 		if (target_velocity[i] != 0){
					if(i == 0 || i == 1){
		 				target_position[0] = current_position[0];
						want_fix[0] = false;
		 				target_position[1] = current_position[1];
						want_fix[1] = false;
						i = 1;
					}
					else{
						target_position[i] = current_position[0];
					}
				}
				else want_fix[i] = true;
			}			
			calculate_out();
		}
		if(print_fatal){
			for(int i = 0; i < 6; i++){
				normal_error[i] = target_position[i] - current_position[i];
			}
			ROS_FATAL("current_position: \t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", current_position[0], current_position[1], current_position[2], current_position[3], current_position[4], current_position[5]);
			ROS_FATAL("target_position: \t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", target_position[0], target_position[1], target_position[2], target_position[3], target_position[4], target_position[5]);
			ROS_FATAL("normal_error: \t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", normal_error[0], normal_error[1], normal_error[2], normal_error[3], normal_error[4], normal_error[5]);
			ROS_FATAL("want_fix: \t\t%d\t%d\t%d\t%d\t%d\t%d", want_fix[0], want_fix[1], want_fix[2], want_fix[3], want_fix[4], want_fix[5]);
			ROS_FATAL("force_output: \t\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", force_output[0], force_output[1], force_output[2], force_output[3], force_output[4], force_output[5]);
			ROS_FATAL("current_velocity:\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", current_velocity[0], current_velocity[1], current_velocity[2], current_velocity[3], current_velocity[4], current_velocity[5]);
			ROS_FATAL("target_velocity:\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf\t%.4lf", target_velocity[0], target_velocity[1], target_velocity[2], target_velocity[3], target_velocity[4], target_velocity[5]);
			ROS_INFO("--------------------------------------------------------------------------------------------------------------------------------------------");
		}
		tell_pub.publish(message_cmd_vel);
		rate.sleep();
	}
}

void calculate_out(){
	distance_xy = sqrt( pow(current_position[0] - target_position[0],2) + pow( current_position[1] - target_position[1] ,2) );
	distance_yaw = check_radian_tan(atan2( target_position[1] - current_position[1] , target_position[0] - current_position[0]));
	diff_yaw = find_min_angular(current_position[5] , distance_yaw);	
	std::cout << "distance xy : " << distance_xy << " yaw : " << distance_yaw << "  diff_yaw : " << diff_yaw << std::endl;
	for(int check = 0 ; check < 6 ; check++ ){
		if(want_fix[check]){
			if(check==0){
				error_position[0] = distance_xy*cos(diff_yaw);
				absolute_error = abs(error_position[0]);
				if(absolute_error < 0.02) force_output[0] = 0;
				else force_output[0] = PID_position[0].calculate_PID(error_position[0], current_velocity[0]);
				if(not same_direction(error_position[check],force_output[check])) force_output[check] = 0; 
				std::cout << "check 0 : " << error_position[0] << " force is : " << force_output[0] << std::endl;
			}
			else if(check==1){
				error_position[1] = distance_xy*sin(diff_yaw);
				absolute_error = abs(error_position[1]);
				if(absolute_error < 0.02) force_output[1] = 0;
				else force_output[1] = PID_position[1].calculate_PID(error_position[1], current_velocity[1]);	
				if(not same_direction(error_position[check],force_output[check])) force_output[check] = 0; 
				std::cout << "check 1 : " << error_position[1] << " force is : " << force_output[1] << std::endl;
			}
			else if(check==2){
				error_position[2] = target_position[2] - current_position[2];
				if(error_position[2] > 0 && error_position[2] < 0.1) force_output[2] = 0;
/*				else if(error_position[2] < 0 && error_position[2] > -0.05){
					force_output[2] = force_z;
					std::cout << "force_z is " << force_z << std::endl;
				}*/
				else force_output[2] = PID_position[2].calculate_PID(error_position[2], current_velocity[2]);
				std::cout << "check 2 : " << error_position[2] << " force is : " << force_output[2] << std::endl;
			}
			else if(check==5){
				error_position[5] = find_min_angular(current_position[5], target_position[5]);
				absolute_error = abs(error_position[5]);
				if(absolute_error<0.02) force_output[5]=0;
				else force_output[5] = PID_position[5].calculate_PID(error_position[5], current_velocity[5]);
				if(not same_direction(error_position[check],force_output[5])) force_output[5] = 0; 
				std::cout << "check "<< check <<" : " << error_position[5] << " force is : " << force_output[5] << std::endl;
			}
			else{
				error_position[check] = find_min_angular(current_position[check], target_position[check]);
				absolute_error = abs(error_position[check]);
				if(absolute_error>-0.05 && absolute_error<0.05) force_output[check]=0;
				else force_output[check] = PID_position[check].calculate_PID(error_position[check], current_velocity[check]);
				if(not same_direction(error_position[check],force_output[check])) force_output[check] = 0; 
				std::cout << "check "<< check <<" : " << error_position[check] << " force is : " << force_output[check] << std::endl;
			}
		}
		else{
//			error_velocity[check] = target_velocity[check] - current_velocity[check];	
//			force_output[check] = PID_velocity[check].calculate_PID(error_velocity[check], -1*current_velocity[check]);			
			force_output[check] = target_velocity[check];
		}
	}
	if(shutdown_force_z){
		force_output[2] = 0;
		force_output[3] = 0;
		force_output[4] = 0;
	}
	if(shutdown_force_xy){
		force_output[1] = 0;
		force_output[0] = 0;
		force_output[5] = 0;
	}
	message_cmd_vel = twist_to_message(force_output);	
}

void shutdown_target_velocity() { for(int i=0 ; i < 6 ; i++) target_velocity[i] = 0; }

void shutdown_force() { for(int i =0 ; i < 6 ; i++) force_output[i] = 0; }

void set_true_fix() { for(int i =0 ; i < 6 ; i++) want_fix[i] = true; }

// 3 max 2 position 1 velocity

void listen_current_state(const nav_msgs::Odometry message){
//	double quaternion[4] = {0,0,0,0} ;
//	message_to_quaternion(message.pose.pose.orientation, *&quaternion);
//	std::cout<< *quaternion << std::endl; 
//	quaternion[0] = message.pose.pose.orientation.x;
//	quaternion[1] = message.pose.pose.orientation.y;
//	quaternion[2] = message.pose.pose.orientation.z;
//	quaternion[3] = message.pose.pose.orientation.w;
	tf::Quaternion quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w);
	tfScalar roll, pitch, yaw;
	tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
	if( start_run || reset_position){
		target_position[0] = message.pose.pose.position.x;
		target_position[1] = message.pose.pose.position.y;
		target_position[2] = message.pose.pose.position.z;
		target_position[3] = 0.0;
		target_position[4] = 0.0;
		target_position[5] = yaw;
		shutdown_force();
		start_run = false;
		reset_position = false;
	}	
	current_position[0] = message.pose.pose.position.x;
	current_position[1] = message.pose.pose.position.y;
	current_position[2] = message.pose.pose.position.z;
	current_position[3] = check_radian_tan((double)roll);
	current_position[4] = check_radian_tan((double)pitch);
	current_position[5] = check_radian_tan((double)yaw);
//	current_position[3] = get_roll_radian(quaternion);
//	current_position[4] = get_pitch_radian(quaternion);
//	current_position[5] = get_yaw_radian(quaternion);
	current_velocity[0] = message.twist.twist.linear.x;	
	current_velocity[1] = message.twist.twist.linear.y;	
	current_velocity[2] = message.twist.twist.linear.z;	
	current_velocity[3] = message.twist.twist.angular.x;	
	current_velocity[4] = message.twist.twist.angular.y;	
	current_velocity[5] = message.twist.twist.angular.z;	
}

void listen_target_velocity(const geometry_msgs::Twist message){
	target_velocity[0] = message.linear.x;
	target_velocity[1] = message.linear.y;
	target_velocity[2] = message.linear.z;
	target_velocity[3] = message.angular.x;
	target_velocity[4] = message.angular.y;
	target_velocity[5] = message.angular.z;
	reset_all_I();
}

void listen_target_position(const geometry_msgs::Point message){
	target_position[0] = message.x;
	target_position[1] = message.y;
	target_position[2] = message.z;
	reset_all_I();
}

void listen_mode(const std_msgs::Int16 message){}

void listen_target_depth(const std_msgs::Float64 message){
	target_position[2] = message.data;
	ROS_DEBUG("In listen depth");
}

void listen_absolute_orientation(const zeabus_controller::orientation message){
	target_position[3] = message.roll;
	target_position[4] = message.pitch;
	target_position[5] = message.yaw;
}

void listen_absolute_yaw(const std_msgs::Float64 message){
	target_position[5] = message.data;
}

void listen_absolute_xy(const zeabus_controller::point_xy message){
	target_position[0] = message.x;
	target_position[1] = message.y;
}

void listen_real_yaw(const std_msgs::Float64 message){
	target_position[5] = current_position[5] - message.data;
	if(target_position[5] < 0) target_position[5]+=2*PI;
	else if(target_position[5] > 2*PI) target_position[5]-=2*PI;
}

void config_constant_PID(zeabus_controller::forcePIDConstantConfig &config, uint32_t level){
	ROS_ERROR("!!!--K changed---!!!");
	Kp_position[0] = config.KPPx;
	Kp_position[1] = config.KPPy;
	Kp_position[2] = config.KPPz;
	Kp_position[3] = config.KPProll;
	Kp_position[4] = config.KPPpitch;
	Kp_position[5] = config.KPPyaw;

	Ki_position[0] = config.KIPx;
	Ki_position[1] = config.KIPy;
	Ki_position[2] = config.KIPz;
	Ki_position[3] = config.KIProll;
	Ki_position[4] = config.KIPpitch;
	Ki_position[5] = config.KIPyaw;
	
	Kd_position[0] = config.KDPx;
	Kd_position[1] = config.KDPy;
	Kd_position[2] = config.KDPz;
	Kd_position[3] = config.KDProll;
	Kd_position[4] = config.KDPpitch;
	Kd_position[5] = config.KDPyaw;
	
	Kp_velocity[0] = config.KPVx;
	Kp_velocity[1] = config.KPVy;
	Kp_velocity[2] = config.KPVz;
	Kp_velocity[3] = config.KPVroll;
	Kp_velocity[4] = config.KPVpitch;
	Kp_velocity[5] = config.KPVyaw;

	Ki_velocity[0] = config.KIVx;
	Ki_velocity[1] = config.KIVy;
	Ki_velocity[2] = config.KIVz;
	Ki_velocity[3] = config.KIVroll;
	Ki_velocity[4] = config.KIVpitch;
	Ki_velocity[5] = config.KIVyaw;
	
	Kd_velocity[0] = config.KDVx;
	Kd_velocity[1] = config.KDVy;
	Kd_velocity[2] = config.KDVz;
	Kd_velocity[3] = config.KDVroll;
	Kd_velocity[4] = config.KDVpitch;
	Kd_velocity[5] = config.KDVyaw;

	Kvs_position[0] = config.KVSx;
	Kvs_position[1] = config.KVSy;
	Kvs_position[2] = config.KVSz;
	Kvs_position[3] = config.KVSroll;
	Kvs_position[4] = config.KVSpitch;
	Kvs_position[5] = config.KVSyaw;
	std::cout << "change PID" << std::endl;
	std::cout << "Kp_position of x is " << Kp_position[0] << std::endl;
	set_all_PID();
	if(not first_time_PID){
//		std::cout << "before save file" << std::endl;
//		PID_file.save_file("Controller");
//		std::cout << "finish save file" << std::endl;
		change_PID = true;
	}
}

std_msgs::Bool is_at_fix_position(double error){}

std_msgs::Bool is_at_fix_orientation(double error){}

double get_roll_radian(double quaternion[4]){
	double result, x, y;
	y = 2*(quaternion[0] * quaternion[1] + quaternion[2]*quaternion[3]);
	x = pow(quaternion[0],2) + pow(quaternion[1],2) - pow(quaternion[2],2) - pow(quaternion[3],2);
	result = atan2(y,x);
	return check_radian_tan(result);
}

double get_yaw_radian(double quaternion[4]){
	double result, x, y;
	y = 2.0*(quaternion[0] * quaternion[3] + quaternion[2]*quaternion[1]);
	x = pow(quaternion[0],2) - pow(quaternion[1],2) - pow(quaternion[2],2) + pow(quaternion[3],2);
	result = atan2(y,x);
	ROS_ERROR("get yaw result is %.5lf",result);
//	std::cout << result << std::endl;
	return check_radian_tan(result);
}

double get_pitch_radian(double quaternion[4]){
	double result = ((-1)*asin( 2 * ( quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2])) * 180 / PI);
	return result;
}

double check_radian_tan(double result){
	double check = result;
	if(check > 0 ) return result;
	else return result + 2*PI;
}

double find_min_angular(double current, double target){
	if(current > target){
		right_yaw = -(current - target);
		left_yaw = (2*PI)+right_yaw; 
	}
	else{
		left_yaw = target-current;
		right_yaw = -((2*PI)-left_yaw);
	}
	double abs_right_yaw = abs(right_yaw);
	double abs_left_yaw = abs(left_yaw);
	if(abs_right_yaw<abs_left_yaw) return right_yaw;
	else return left_yaw;
}

geometry_msgs::Twist twist_to_message(double twist[6]){
	geometry_msgs::Twist answer;
	answer.linear.x = twist[0];
	answer.linear.y = twist[1];
	answer.linear.z = twist[2];
	answer.angular.x = twist[3];
	answer.angular.y = twist[4];
	answer.angular.z = twist[5];
	return answer;
}

void message_to_quaternion(const geometry_msgs::Quaternion message, double* quaternion){
	*quaternion = message.x;
	*(quaternion+1) = message.y;
	*(quaternion+2) = message.z;
	*(quaternion+3) = message.w;
	
}

void set_all_PID(){
	for(int count = 0 ; count < 6 ; count++){
		PID_position[count].set_PID(Kp_position[count], Ki_position[count], Kd_position[count], Kvs_position[count]);
		PID_velocity[count].set_PID(Kp_velocity[count], Ki_velocity[count], Kd_velocity[count], 0);
	}
	reset_all_I();
}

void reset_all_I(){
	for(int check = 0 ; check < 6 ; check++){
		PID_position[check].reset_I();
		PID_velocity[check].reset_I();
	}
}

int found_sign(double direction){
	if(direction < 0) return -1;
	else return 1;
}

double abs(double a){
	if(a < 0) return a*(-1);
	return a;
}

int abs(int a){
	if(a < 0) return a*(-1);
	return a;
}

void test_current_state(const geometry_msgs::Point message){
	current_position[0] = message.x;
	current_position[1] = message.y;
	current_position[2] = message.z;
}

void test_current_orientation(const zeabus_controller::orientation message){
	current_position[3] = message.roll;
	current_position[4] = message.pitch;
	current_position[5] = message.yaw;
}

bool same_direction(double num_01, double num_02){
	if(num_01*num_02 >= 0) return true;
	else return false;
}
