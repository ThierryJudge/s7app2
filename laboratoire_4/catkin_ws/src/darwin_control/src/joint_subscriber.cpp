#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

void chatterCallback(const JointStateConstPtr &msg)
{
  ROS_INFO("[ position: %f ]", msg->position.at(0));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darwin_Joint_Subscriber");
  ros::NodeHandle n;

  std::string subscription = "/darwin/j_pelvis_r_position_controller/state";

  ROS_INFO("Subscribing to %s", subscription.c_str());
  ros::Subscriber sub = n.subscribe(subscription.c_str(), 1000, chatterCallback);
  
  ros::spin();

  return 0;
}
