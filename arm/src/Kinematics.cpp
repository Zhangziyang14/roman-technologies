#include "ros/ros.h"
#include "nero_msgs/InverseKinematica.h"
#include "nero_msgs/ArmJoint.h"
#include <geometry_msgs/Pose.h>

#define UPPER_ARM_LENGTH 	0.36		//Length of upper arm in [m]
#define WRIST_LENGTH 		0.18		//Length of wrist in [m]
#define EFFECTOR_ORIGIN 	0.18		//Offset in [m] from wrist to the point 3 [cm] in front of the gripper
#define MID_ARM_LENGTH		0.18		//Length of the link in the middle that stays horizontal
ros::Publisher *coordinate_publisher;
double currentX, currentY, currentZ;

/*
 * Calculates the joint position of the shoulder with a given z value in the gripper's configuration space
 */
double calculateShoulderJointPosition(double z)
{
	return std::asin(z/UPPER_ARM_LENGTH);
}

/*
 * Calculates the joint position of the wrist with a given x value in the gripper's configuration space
 */
double calculateWristJointPosition(double x)
{
	// Multiply with -1 to flip x-axis
	return -1*std::asin(x/(WRIST_LENGTH + EFFECTOR_ORIGIN));
}

/*
 * Returns x position of the gripper in configuration space
 */
double calculateCurrentXPosition(double beta)
{
	return std::sin(-beta)*(WRIST_LENGTH + EFFECTOR_ORIGIN);
}

/*
 * Returns y position of the gripper in configuration space
 */
double calculateCurrentYPosition(double alpha, double beta)
{
	return std::cos(alpha)*UPPER_ARM_LENGTH + MID_ARM_LENGTH + std::cos(beta)*(WRIST_LENGTH+EFFECTOR_ORIGIN);
}

/*
 * Returns z position of the gripper in configuration space
 */
double calculateCurrentZPosition(double alpha)
{
	return std::sin(alpha)*UPPER_ARM_LENGTH;
}

/*
 * Solves inverse kinematics for the arm at request, returns joint positions for arm motors.
 */
bool solveIK(nero_msgs::InverseKinematica::Request& request, nero_msgs::InverseKinematica::Response& response)
{
	response.configuration.upper_joint 	= calculateShoulderJointPosition(request.target.position.z);
	response.configuration.wrist_joint 	= calculateWristJointPosition(request.target.position.x);
	return true;
}

/*
 * Solves forward kinematics for the arm, calculates coordinates in the gripper's configuration space with the current joint positions.
 */
void ForwardKinematicsCB(const nero_msgs::ArmJoint &msg)
{
	geometry_msgs::Pose coordinate_msg;
	currentZ = calculateCurrentZPosition(msg.upper_joint);
	currentX = calculateCurrentXPosition(msg.wrist_joint);
	currentY = calculateCurrentYPosition(msg.upper_joint, msg.wrist_joint);

	coordinate_msg.position.x = currentX;
	coordinate_msg.position.y = currentY;
	coordinate_msg.position.z = currentZ;

	coordinate_publisher->publish(coordinate_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Kinematics");
	ros::NodeHandle nodeHandle;

	coordinate_publisher = new ros::Publisher(nodeHandle.advertise<geometry_msgs::Pose>("/armCoordinatePositionFeedbackTopic", 1));
	ros::Subscriber position_sub = nodeHandle.subscribe("/armJointPositionFeedbackTopic", 1, ForwardKinematicsCB);

	ros::ServiceServer service = nodeHandle.advertiseService("IK", solveIK);

	ROS_INFO("Kinematics node ready");

	int sleep_rate;
	nodeHandle.param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}

	return 0;
}
