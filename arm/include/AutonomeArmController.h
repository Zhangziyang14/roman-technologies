#ifndef AutonomeArmController_h
#define AutonomeArmController_h

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif

#include <ros/ros.h>
#include "arm/armJointPos.h"
#include "arm/armCoordinatesPos.h"
#include "arm/IK.h"

#define MIN_Z_VALUE -36
#define MAX_Z_VALUE 16.0
#define MIN_X_VALUE -29.4
#define MAX_X_VALUE 33.4


class AutonomeArmController
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	ros::Publisher mJointCommandPublisher;		//Publishes commands for ArmMotorHandler
	ros::Subscriber mCurrentPositionSubscriber;	//Listens to current positions in coordinates of configuration space
	ros::Subscriber mCommandSubscriber;			//Listens to commands in coordinates
	ros::ServiceClient mKinematicsClient;		//Client to call for service to calculate inverse kinematics

	arm::armCoordinatesPos mCurrentPos;			//Keeps track of current position in coordinates of configuration space

public:
	AutonomeArmController(): mNodeHandle(""){}
	~AutonomeArmController()
	{
		mNodeHandle.shutdown();
	}

	void init();
	void armPositionCB(const arm::armCoordinatesPos &msg);
	void cmdCB(const arm::armCoordinatesPos &msg);
};

#endif
