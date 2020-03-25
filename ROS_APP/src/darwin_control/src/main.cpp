/*

Code for GEI744, ROS and Darwin_Op robotic plateform.
Author: Guillaume Durandau, Guillaume.Durandau@Usherbrooke.ca
License: BSD

Vers: 0.0.1
Date: 05/02/14

*/

#include "ros/ros.h"
#include "lib/DarwinJointControl.h"
#include "lib/DarwinReadFile.h"
/*
	Joint : j_pelvis_r, j_thigh1_r, j_thigh2_r, j_tibia_r, j_ankle1_r, j_ankle2_r
*/

int main(int argc, char **argv)
{
	/**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "Darwin_Joint_Control");

	/**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle node;
	DarwinJointControl darwin(&node);
	// darwin.setJoint("j_high_arm_r", M_PI_2);
	DarwinReadFile table("qs.txt");

	ROS_INFO("%f", table.getTime(0));
	ROS_INFO("%f", table.getTime(1));
	ROS_INFO("%f", table.getTime(2));
	ROS_INFO("%f", table.getTime(3));
	ROS_INFO("%f", table.getJoint(3, 0));
	ROS_INFO("%f", table.getJoint(3, 2));

	ros::Duration rate_duration(1.0 / 25.0);

	ros::Rate rate(25);

	int pCounter = 0;

	// Tentative d'atteindre la position initiale plus lentement.
	ros::Duration initialMovementDuration(2);

	double pelvis, thigh1, thigh2, tibia, ankle1, ankle2;

	pelvis = table.getJoint(pCounter, 0);
	thigh1 = table.getJoint(pCounter, 1);
	thigh2 = table.getJoint(pCounter, 2);
	tibia = table.getJoint(pCounter, 3);
	ankle1 = table.getJoint(pCounter, 4);
	ankle2 = table.getJoint(pCounter, 5);

	// darwin.goalPos("j_pelvis_r", pelvis, initialMovementDuration, rate);
	// darwin.goalPos("j_thigh1_r", thigh1, initialMovementDuration, rate);
	// darwin.goalPos("j_thigh2_r", thigh2, initialMovementDuration, rate);
	// darwin.goalPos("j_tibia_r", tibia, initialMovementDuration, rate);
	// darwin.goalPos("j_ankle1_r", ankle1, initialMovementDuration, rate);
	// darwin.goalPos("j_ankle2_r", ankle2, initialMovementDuration, rate);

	ROS_INFO("Placing DarwinOP to initial position");

	// darwin.setJoint("j_pelvis_r", pelvis);
	darwin.setJoint("j_thigh1_r", thigh1);
	darwin.setJoint("j_thigh2_r", thigh2);
	darwin.setJoint("j_tibia_r", tibia);
	darwin.setJoint("j_ankle1_r", ankle1);
	darwin.setJoint("j_ankle2_r", ankle2);

	// initialMovementDuration.sleep();

	ROS_INFO("Starting movement");
	
	rate.reset();
	while (ros::ok())
	{
		// j_pelvis_r, j_thigh1_r, j_thigh2_r, j_tibia_r, j_ankle1_r, j_ankle2_r
		pelvis = table.getJoint(pCounter, 0);
		thigh1 = table.getJoint(pCounter, 1);
		thigh2 = table.getJoint(pCounter, 2);
		tibia = table.getJoint(pCounter, 3);
		ankle1 = table.getJoint(pCounter, 4);
		ankle2 = table.getJoint(pCounter, 5);

		ROS_INFO("\nValues read from file, counter = %d", pCounter);
		ROS_INFO("pelvis, read: %f", pelvis);
		ROS_INFO("thigh1, read: %f", thigh1);
		ROS_INFO("thigh2, read: %f", thigh2);
		ROS_INFO("tibia, read: %f", tibia);
		ROS_INFO("ankle1, read: %f", ankle1);
		ROS_INFO("ankle2, read: %f\n\n", ankle2);

		darwin.setJoint("j_pelvis_r", pelvis);
		darwin.setJoint("j_thigh1_r", thigh1);
		darwin.setJoint("j_thigh2_r", thigh2);
		darwin.setJoint("j_tibia_r", tibia);
		darwin.setJoint("j_ankle1_r", ankle1);
		darwin.setJoint("j_ankle2_r", ankle2);

		if (pCounter < 49)
			pCounter++;
		else
			break;

		rate.sleep();
	}

	// //	darwin->getNameOnScreen();

	// 	ros::Duration time_tot(10.0);
	// 	ros::Rate rate(40);
	// 	//*
	// 	darwin.goalPos("j_pan", 1.5, time_tot, rate);

	// 	darwin.goalPos("j_pelvis_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_thigh1_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_thigh2_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_tibia_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_ankle1_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_ankle2_l", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_pelvis_r", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_thigh1_r", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_thigh2_r", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_tibia_r", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_ankle1_r", 	0.0, time_tot, rate);
	// 	darwin.goalPos("j_ankle2_r", 	0.0, time_tot, rate);
	// 	//*/
	// 	time_tot.sleep();

	// 	darwin.goalPos("j_ankle2_r", 0.2, time_tot, rate);
	// 	darwin.goalPos("j_ankle2_l", 0.2, time_tot, rate);
	// 	darwin.goalPos("j_thigh1_l", 0.2, time_tot, rate);
	// 	darwin.goalPos("j_thigh1_r", 0.2, time_tot, rate);

	// 	time_tot.sleep();

	/*
	j_urdf["L_SHOULDER_PITCH"] = "j_shoulder_l";
	j_urdf["L_SHOULDER_ROLL"] = "j_high_arm_l";
	j_urdf["L_ELBOW"] = "j_low_arm_l";
	j_urdf["R_SHOULDER_PITCH"] = "j_shoulder_r";
	j_urdf["R_SHOULDER_ROLL"] = "j_high_arm_r";
	j_urdf["R_ELBOW"] = "j_low_arm_r";
	j_urdf["HEAD_PAN"] = "j_pan";
	j_urdf["HEAD_TILT"] = "j_tilt";
	j_urdf["L_HIP_YAW"] = "j_pelvis_l";
	j_urdf["L_HIP_ROLL"] = "j_thigh1_l";
	j_urdf["L_HIP_PITCH"] = "j_thigh2_l";
	j_urdf["L_ANKLE_PITCH"] = "j_tibia_l";
	j_urdf["L_ANKLE_ROLL"] = "j_ankle1_l";
	j_urdf["L_KNEE"] = "j_ankle2_l";
	j_urdf["R_HIP_YAW"] = "j_pelvis_r";
	j_urdf["R_HIP_ROLL"] = "jthigh1_r";
	j_urdf["R_HIP_PITCH"] = "j_thigh2_r";
	j_urdf["R_ANKLE_PITCH"] = "j_tibia_r";
	j_urdf["R_ANKLE_ROLL"] = "j_ankle1_r";
	j_urdf["R_KNEE"] = "j_ankle2_r"; 
	*/

	/*	ros::Duration time(2.0);
	while (ros::ok())
	{	
		darwin.setJoint("j_high_arm_r", 0.2);
		ros::spinOunce();
		time.sleep();
		darwin.setJoint("j_high_arm_r", 0.0);	
	}*/
	return 0;
}
