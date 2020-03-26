#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"


typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

//Define the joint state
sensor_msgs::JointState joint_state;

    // ROS_INFO("Joint <<%d>>'s position -> %f", joint_state.name[i].c_str(), joint_state.position[i]);

// [j_ankle1_l, j_ankle1_r, j_ankle2_l, j_ankle2_r, j_gripper_l, j_gripper_r, j_high_arm_l,
//   j_high_arm_r, j_low_arm_l, j_low_arm_r, j_pan, j_pelvis_l, j_pelvis_r, j_shoulder_l,
//   j_shoulder_r, j_thigh1_l, j_thigh1_r, j_thigh2_l, j_thigh2_r, j_tibia_l, j_tibia_r,
//   j_tilt, j_wrist_l, j_wrist_r]





void chatterCallback(const JointStateConstPtr &msg)
{

  // joint_state.position[0] = msg->position.at(0);
  // joint_state.position[1] = msg->position.at(1);
  // joint_state.position[2] = msg->position.at(2);
  // joint_state.position[3] = msg->position.at(3);
  // joint_state.position[4] = msg->position.at(4);
  // joint_state.position[5] = msg->position.at(5);

  for(int i = 0; i<=5; i++)
    ROS_INFO("Joint <<%d>>'s position -> %f", i, msg->position.at(i));
  
    // ROS_INFO("Joint <<%d>>'s position -> %f", joint_state.name[i].c_str(), joint_state.position[i]);
}

void JointStateCallback(const control_msgs::JointControllerStateConstPtr &msg)
{
    ROS_INFO("Joint's position -> %f", msg->process_value);
  
    // ROS_INFO("Joint <<%d>>'s position -> %f", joint_state.name[i].c_str(), joint_state.position[i]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Darwin_Joint_Subscriber");
  ros::NodeHandle n;


  // joint_state.name[0] = "j_pelvis_r";
  // joint_state.name[1] = "j_thigh1_r";
  // joint_state.name[2] = "j_thigh2_r";
  // joint_state.name[3] = "j_tibia_r";
  // joint_state.name[4] = "j_ankle1_r";
  // joint_state.name[5] = "j_ankle2_r";

  // joint_state.position[0] = 0.0;
  // joint_state.position[1] = 0.0;
  // joint_state.position[2] = 0.0;
  // joint_state.position[3] = 0.0;
  // joint_state.position[4] = 0.0;
  // joint_state.position[5] = 0.0;

  // std::string subscription = "/darwin/joint_states";
  std::string subscription = "/darwin/j_pan_position_controller/state";


  ROS_INFO("Subscribing to %s", subscription.c_str());
  ros::Subscriber sub = n.subscribe(subscription.c_str(), 1000, JointStateCallback);

  ros::spin();

  return 0;
}
