
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>

class odometria 
{
public:
	odometria();
	void spin();

private:
	ros::NodeHandle 			n;
	ros::Publisher 				odom_pub;
	ros::Publisher 				joint_pub;
	tf::TransformBroadcaster 	odom_broadcaster;
	ros::Time 					current_time;
	ros::Time 					last_time;
	ros::Subscriber 			wheel_velocities;	

	sensor_msgs::JointState 			joint_state;
	geometry_msgs::TransformStamped 	odom_trans;

	geometry_msgs::Vector3Stamped 		rpm_msg;

	ros::Duration 		t_delta;
	ros::Time 			t_next;
	ros::Time 			then;

	double x;
	double y;
	double th;

	double vx;
	double vy;
	double vth;

	double dt;
	double delta_x;
	double delta_y;
	double delta_th;

	double last_vel_izq;
	double last_vel_der;
	double distance_left;
	double distance_right;

	double dx,dr;
	double x_final;
	double y_final;
	double theta_final;

	double left_vel;
	double right_vel;

	double rate;

	void init_variables();
	void update();
	void get_velocities(const geometry_msgs::Vector3Stamped& vel);
	
};

odometria::odometria()
{
	init_variables();
	ROS_INFO("Iniciando el calculo de odometria");
	
	odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);

	joint_pub = n.advertise<sensor_msgs::JointState>("camera_link",1);
	
	wheel_velocities = n.subscribe("/rpm",10, &odometria::get_velocities, this);

}

void odometria::init_variables()
{
	//start variables from origin
	x = 0.0;
	y = 0.0;
	th= 0.0;

	//es necesatio calcular los siguientes valores en base a las velocidades leidos por el encoder 

	vx 	= 0.1; 	//		m/s
	vy 	= 0.1;
	vth	= 0.1;
	//rate publishing
	rate=50;

	last_vel_izq = left_vel;
	last_vel_der = right_vel;

	x_final = 0;
	y_final = 0;
	theta_final = 0;

	t_delta = ros::Duration(1.0/rate);
	t_next = ros::Time::now() + t_delta;
	then = ros::Time::now();

	current_time = ros::Time::now();
	last_time = ros::Time::now();
}

void odometria::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

void odometria::update()
{
	
	double speed_left,speed_right,d_left ,d_right ,d ,th ,x ,y;
	double radius = 0.0575, pi_2 = 6.283185307;
	double base_width = 0.33736;

	current_time = ros::Time::now();

	if (current_time > t_next)
	{
		dt = (current_time - last_time).toSec();

		speed_left = (radius * pi_2) / 60;
		double rpm_izq = 0.5 *(left_vel + last_vel_izq);
		d_left = rpm_izq * speed_left * dt ;

		speed_right = (radius * pi_2) / 60;
		double rpm_der = 0.5 *(right_vel + last_vel_der);
		d_right = rpm_der * speed_right * dt;

		//ROS_INFO_STREAM("  distancia izquierda: "<< distance_left<<" d left: "<<d_left);

		d = (d_left + d_right) / 2;
		th = (d_right - d_left) / base_width;

		dx = d / dt;
		dr = th / dt;

		if (d != 0)
		{
			x = cos(th)*d;
			y = -sin(th)*d;
			x_final = x_final + ( cos(theta_final)*x - sin(theta_final)*y );
			y_final = y_final + ( sin(theta_final)*x + cos(theta_final)*y );
		}

		if(th !=0) 
		{
			theta_final = theta_final + th ;
		}

		geometry_msgs::Quaternion odom_quat;
		odom_quat.x = 0;
		odom_quat.y = 0;
		odom_quat.z = 0;

		odom_quat.z = sin(theta_final / 2);
		odom_quat.w = cos(theta_final / 2);
		
		geometry_msgs::TransformStamped odom_trans;
		
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x_final;
		odom_trans.transform.translation.y = y_final;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = x_final;
		odom.pose.pose.position.y = y_final;
		odom.pose.pose.position.z = 0;
		odom.pose.pose.orientation = odom_quat;

		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = dx ;
		odom.twist.twist.linear.y = 0 ;
		odom.twist.twist.angular.z = dr;

		odom_pub.publish(odom);

		last_time = current_time;
		last_vel_izq = left_vel;
		last_vel_der = right_vel;
		ros::spinOnce();	
	}
}

void odometria::get_velocities(const geometry_msgs::Vector3Stamped& vel)
{
	left_vel = (double)vel.vector.x;
	right_vel = (double)vel.vector.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"odometria");
	odometria obj;
	obj.spin();
	return 0;
}
