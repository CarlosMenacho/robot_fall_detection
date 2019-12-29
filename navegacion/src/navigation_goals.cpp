#include <ros/ros.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient ac("move_base", true);

void goal_callback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
     
  }


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Subscriber goal_sub = n.subscribe("/robot/nav", 10, goal_callback );

  //wait for the action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}

  while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}

