/*
 * io_and_pressure.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: zeabus3
 */
    /*
 * io_and_pressure.cpp
 *
 *  Created on: Jun 15, 2015
 *      Author: SaringkarnP
 */
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <modbus_ascii_ros/IOCommand.h>

#include <modbus_ascii_ros/Switch.h>


#include <mblib/MBDevice.h>
#include <termios.h>
#include <vector>

#include <boost/thread.hpp>

#define GPIO_ADDR 18
#define ATMOSPHORIC_PRESSURE    14.7
#define ON 1
#define OFF 0

extern int show_debug_info;

boost::mutex mutex;
ros::NodeHandle* nhPtr;
MBSlave* mb_slave_gpio;

boost::mutex g_mutex;
ros::Publisher g_pub_baro;
ros::Publisher g_pub_baro_pose;
ros::Publisher g_pub_baro_twist;
ros::Publisher g_pub_baro_odom;

ros::Publisher g_pub_switch;
ros::Subscriber g_sub_switch;

static int motor_sw_status = OFF;

int fd = 0;

bool IO_ON(modbus_ascii_ros::IOCommand::Request  &req,
             modbus_ascii_ros::IOCommand::Response &res)
{
	show_debug_info = 1;
  boost::mutex::scoped_lock(g_mutex);
  ROS_INFO_STREAM(req);
  int ret;
  ret = mb_slave_gpio->Write_single_coil(req.channel, 1);
  res.result = (ret == 1) ? true : false;
show_debug_info = 0;
  return res.result;
}

bool IO_OFF(modbus_ascii_ros::IOCommand::Request  &req,
             modbus_ascii_ros::IOCommand::Response &res)
{
	  boost::mutex::scoped_lock(g_mutex);
	  ROS_INFO_STREAM(req);
	  int ret;
	  ret = mb_slave_gpio->Write_single_coil(req.channel, 0);
	  res.result = (ret == 1) ? true : false;
	  return res.result;
}


double moveavg(double a) {
	static int winsize=80;
	static int pose=0;
	static double sum=0;
	static double window_vel[100]={0};
	static double alpha = 0.1;
	static double accumulator = 0;
	static int check = 1;
    int n;
    //printf("del%lf %d %d\n",window_vel[pose%winsize],pose,winsize);

    sum-=window_vel[pose%winsize];
    sum+=a;
    window_vel[pose%winsize]=a;
    pose++;
    if(pose>winsize) n=winsize;
    else n=pose;
    //pose=pose%winsize+winsize;
    return sum/n;
    /*
    if (check == 1) {
	accumulator = a;
	check = 0;
    }
    accumulator = (alpha * a ) + (1.0 - alpha) * accumulator ;
    return accumulator;
    */
}


int main(int argc, char **argv)
{
	  ros::init(argc, argv, "io_and_pressure");
	  ros::NodeHandle nh("~");

	  std::string device;
	  int baudrate;
	  double atm_pressure;
	  double depth_offset;
	  nh.param<double>("atm_pressure", atm_pressure, 15.6);
	  nh.param<double>("depth_offset", depth_offset, 0);
	  uint16_t control_word[2];
	  std::string frame_id;
	  show_debug_info = 0;
	  nh.param<std::string>("io_device", device, "/dev/ttyUSB10");
	  nh.param<std::string>("frame_id", frame_id, "baro_link");

	  int fd = 0;


	  if(Open_port(&fd, device.c_str(), B115200, 7, 0, 2, 0) == 0)
	  {
	    ROS_FATAL("Cannot open: %s", device.c_str());
	    return -1;
	  }


	  mb_slave_gpio = new MBSlave(GPIO_ADDR);
	  mb_slave_gpio->Init(fd);



	  g_pub_baro = nh.advertise<sensor_msgs::FluidPressure>("/baro/pressure", 10);
	  g_pub_baro_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/baro/pose", 10);
	  g_pub_baro_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/baro/data", 10);
	  g_pub_baro_odom = nh.advertise<nav_msgs::Odometry>("/baro/odom", 10);


	 ros::ServiceServer service_mb_IO_ON = nh.advertiseService("IO_ON", IO_ON);
	ros::ServiceServer service_mb_IO_OFF = nh.advertiseService("IO_OFF", IO_OFF);	
	  g_pub_switch = nh.advertise<modbus_ascii_ros::Switch>("/switch/data", 10);
	  //g_sub_switch = nh.subscribe<modbus_ascii_ros::Switch>("/switch/data", 10, switchMonitorCallBack);

	  int baro_and_switch_rate = 50;
	  int interval_count = 0;
	  int tick = 0;
	  ros::Rate rate(baro_and_switch_rate);

	  sensor_msgs::FluidPressure pressure;
	  double last_zaxis = 0;
	  geometry_msgs::TwistWithCovarianceStamped twist;
	  nav_msgs::Odometry odom;

	  geometry_msgs::PoseWithCovarianceStamped pose;

	  pose.header.frame_id = "odom";


	  pressure.header.frame_id = frame_id;
	  twist.header.frame_id = frame_id;
	  odom.header.frame_id = "odom";
	  odom.child_frame_id = "base_link";

	  pose.pose.covariance[14] = 0;
	  twist.twist.covariance[14] = 0.04;
	  odom.pose.covariance[14] = 0.2;

	  modbus_ascii_ros::Switch switch_data;

	  double period;
  while(ros::ok())
    {
      uint16_t gpio_status[2];
      uint16_t baro;
      int ret;
      //uint8_t switches = 0;
      uint16_t switches = 0;

      if (interval_count >= baro_and_switch_rate)
      {
        //Led blink
        interval_count = 0;
      }
      interval_count++;
      ret = 0;
      {
        boost::mutex::scoped_lock(g_mutex);
        //read baro
      //show_debug_info = 1;
        ret = mb_slave_gpio->Read_input_registers(0, 2, gpio_status);
        //show_debug_info = 0;
        if(ret)
	{
		//ROS_INFO("Pressure read.");
		  period = rate.expectedCycleTime().toSec();
	}else{
		ROS_ERROR_STREAM("Pressure read error.");
		period += rate.expectedCycleTime().toSec();
		  rate.sleep();
		  ros::spinOnce();
		  continue;

	  }

        //read switch
        baro = gpio_status[0];
        switches = gpio_status[1];

        //mb_slave_gpio->Read_discrete_inputs(0, 3, &switches);
      }

      //ROS_INFO("%d 0x%02x", baro, switches);

      //publish baro as z twist
      twist.header.stamp = ros::Time::now();
      odom.header.stamp = ros::Time::now();
      pose.header.stamp = ros::Time::now();

      /*
       * pressure sensor
       * Measurement range 0 - 15 psi (gauged psi)
       * 0.5 - 4.5V
       * A2D 10 bits
       * 1.4579 psi/m
       */

      //double avg_baro=moveavg((double)baro);
      double psi = (( ((baro / 65536.0) * 5.0) - 0.5) / (4.0) * 30.0); // change avg_baro to baro 9.07.15 11.20
      pressure.fluid_pressure = 6894.75729 * (psi - atm_pressure); //Pa
      double depth = ( (psi-atm_pressure) * 0.6382978723  ) + depth_offset;//   old scale 1.4579

      //depth = 2.5;
      double zaxis = -depth;
      if ( tick % baro_and_switch_rate * 5 == 0){
          printf("d:%6.3lf psi:%6.3f raw:%d v:%6.3f\n",depth,psi,baro,baro/65536.0*5);
      }
      tick++ ;
      tick %= baro_and_switch_rate * 5;

      double dz = zaxis - last_zaxis;
      if (abs(dz) > 0.5){
  	dz = 0;
      }
      //printf("dz:%6.3f\n",dz);


      last_zaxis = zaxis;

      double tmp = rate.expectedCycleTime().toSec(); //0.01;//rate.cycleTime().toSec(); // expectedcycletime()
      double vz = dz / period / 2;
      twist.twist.twist.linear.z = vz;
      odom.pose.pose.position.z = zaxis;
      pose.pose.pose.position.z = zaxis;
      //printf("time:%6.3 vz:%6.3f\n" , tmp, vz);

      if(g_pub_baro.getNumSubscribers())
        g_pub_baro.publish(pressure);
      if(g_pub_baro_twist)
        g_pub_baro_twist.publish(twist);
      if(g_pub_baro_odom)
        g_pub_baro_odom.publish(odom);
      if(g_pub_baro_pose)
          g_pub_baro_pose.publish(pose);


      //publish switch
      switch_data.header.stamp = twist.header.stamp;

      switch_data.motor_switch  = switches & 0x02;
      switch_data.auto_switch   = switches & 0x04;
      switch_data.system_switch = switches & 0x01;

      motor_sw_status = (switches & 0x02)?ON:OFF;
      if(g_pub_switch.getNumSubscribers())
        g_pub_switch.publish(switch_data);


      rate.sleep();
      ros::spinOnce();
    }

    return 0;
  }





