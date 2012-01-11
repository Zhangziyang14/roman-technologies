#ifndef AutonomeArmController_h
#define AutonomeArmController_h

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif

#include <ros/ros.h>
#include "arm/armJointPos.h"
#include <geometry_msgs/Pose.h>
#include "arm/IK.h"

#define MIN_Z_VALUE -32.1
#define MAX_Z_VALUE 17.25
#define MIN_X_VALUE -20
#define MAX_X_VALUE 35.95


class AutonomeArmController
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	ros::Publisher mJointCommandPublisher;		//Publishes commands for ArmMotorHandler
	//ros::Subscriber mCurrentPositionSubscriber;	//Listens to current positions in coordinates of configuration space
	ros::Subscriber mCommandSubscriber;			//Listens to commands in coordinates
	ros::ServiceClient mKinematicsClient;		//Client to call for service to calculate inverse kinematics

	geometry_msgs::Pose mCurrentPos;			//Keeps track of current position in coordinates of configuration space

public:
	AutonomeArmController(): mNodeHandle(""){}
	~AutonomeArmController()
	{
		mNodeHandle.shutdown();
	}

	void init();
	//void armPositionFeedbackCB(const geometry_msgs::Pose &msg);
	void cmdCB(const geometry_msgs::Pose &msg);
};

#endif
