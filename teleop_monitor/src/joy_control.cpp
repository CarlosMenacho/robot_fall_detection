#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Teleop
{
public:
  Teleop();
  void spin();

private:

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh;
  
  ros::Publisher set_point;
  ros::Subscriber joy_sub;

  geometry_msgs::Twist vels;

  double vel_x;
  double vel_y;
  double rate;
  void update();
  void init_variables();
  
};

Teleop::Teleop()
{
  init_variables();
  set_point = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
}

void Teleop::init_variables()
{
  vel_x = 0;
  vel_y = 0;
  rate = 10;
}

void Teleop::spin()
{
  ros::Rate loop_rate(rate);
  while(ros::ok())
  {
    update();
    loop_rate.sleep();
  }
}

void Teleop::update()
{
  geometry_msgs::Twist vels;
  vels.linear.x =  vel_x; //0.2 * std::tan(1.4*vel_x) ;
  vels.angular.z = vel_y;
  set_point.publish(vels);
  ros::spinOnce();
}


void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  vel_x = 0.5 * joy->axes[1]; //1 
  vel_y = 0.5 * joy->axes[3]; // 3
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  Teleop teleop_turtle;
  teleop_turtle.spin();
  return 0;
}
