#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class driver
{
public:
	driver();
	void spin();
private:
	ros::NodeHandle n;
	ros::Publisher left_motor;
	ros::Publisher right_motor;
	ros::Subscriber cmd_vel_sub;

	float left;
	float right;

	double rate;

	void init_variables();
	void get_parameters();
	void spinOnce();
	void twistCallback( const geometry_msgs::Twist &twist_aux);

};
	
driver::driver()
{
	init_variables();
	get_parameters();
	ROS_INFO("ajustando controlador de motores... ");
	ROS_INFO("done.");
	cmd_vel_sub = n.subscribe("cmd_vel", 10 , &driver::twistCallback, this);
	left_motor = n.advertise<std_msgs::Int64>("left_motor",50);
	right_motor = n.advertise<std_msgs::Int64>("right_motor",50);
}
void driver::init_variables()
{
	left = 0;
	right = 0;
	rate = 0;
}
void driver::get_parameters()
{
	if(n.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate from param: ",rate);
	}
}
void driver::spinOnce()
{
	
}