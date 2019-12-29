
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

class marker 
{
public:
  marker();
  void spin();

private:
  ros::NodeHandle       n;
  ros::Publisher        marker_pub;
  ros::Subscriber       position;
  ros::Time             current_time;


  visualization_msgs::Marker marker_point;

  double left_vel;
  double right_vel;

  double rate;

  uint32_t shape; 

  void init_variables();
  void update();
  void get_velocities(const geometry_msgs::Vector3Stamped& vel);
  
};

marker::marker()
{
  init_variables();
  ROS_INFO("marker visualization ");
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
}


void marker::init_variables()
{

  shape = visualization_msgs::Marker::CUBE;
  current_time = ros::Time::now();

}

void marker::spin()
{
  ros::Rate loop_rate(rate);
  while(ros::ok())
  {
    update();
    loop_rate.sleep();
  }
}

void marker::update()
{
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_point.header.frame_id = "/odom";
    marker_point.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_point.ns = "basic_shapes";
    marker_point.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_point.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_point.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_point.pose.position.x = 0;
    marker_point.pose.position.y = 0;
    marker_point.pose.position.z = 0;
    marker_point.pose.orientation.x = 0.0;
    marker_point.pose.orientation.y = 0.0;
    marker_point.pose.orientation.z = 0.0;
    marker_point.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_point.scale.x = 1.0;
    marker_point.scale.y = 1.0;
    marker_point.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_point.color.r = 0.0f;
    marker_point.color.g = 1.0f;
    marker_point.color.b = 0.0f;
    marker_point.color.a = 1.0;

    marker_point.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {

      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker_point);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

}

void marker::get_velocities(const geometry_msgs::Vector3Stamped& vel)
{
  //ROS_INFO("velocidad adquirida");

  left_vel = (double)vel.vector.x;
  right_vel = (double)vel.vector.y;

  //ROS_INFO_STREAM(" velocidad izquierda: " << left_vel);
  //ROS_INFO_STREAM(" velocidad derecha: "   << right_vel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"marker");
  marker obj;
  obj.spin();
  return 0;
}