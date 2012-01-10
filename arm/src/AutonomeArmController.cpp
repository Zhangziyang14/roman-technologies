#include "AutonomeArmController.h"
/*
 *	Listens to commands and translates it to joint commands for ArmMotorHandler using inverse kinematics
 */
void AutonomeArmController::cmdCB(const arm::armCoordinatesPos &msg)
{
	arm::IK srv;

	//Listen to command
	srv.request.target = msg;

	//Check constraints
	if(msg.z_value > MAX_Z_VALUE)
	{
		ROS_WARN("REACHED MAX_Z_VALUE");
		srv.request.target.z_value = MAX_Z_VALUE;
	}
	else if(msg.z_value < MIN_Z_VALUE)
	{
		ROS_WARN("REACHED MAX_Z_VALUE");
		srv.request.target.z_value = MIN_Z_VALUE;
	}

	if(msg.x_value > MAX_X_VALUE)
	{
		ROS_WARN("REACHED MAX_X_VALUE");
		srv.request.target.x_value = MAX_X_VALUE;
	}
	else if(msg.x_value < MIN_Z_VALUE)
	{
		ROS_WARN("REACHED MAX_X_VALUE");
		srv.request.target.x_value = MIN_X_VALUE;
	}

	//Calculate joint configurations with IK solver
	if((srv.request.target.z_value <= MAX_Z_VALUE) && (srv.request.target.z_value >= MIN_Z_VALUE) && (srv.request.target.x_value <= MAX_X_VALUE) && (srv.request.target.x_value <= MAX_X_VALUE))
	{
		mKinematicsClient.call(srv);
	}
	//Call trajectory planner service?

	arm::armJointPos cmd_msg;

	//Send command to Motorhandler
	cmd_msg.upper_joint = srv.response.configuration.upper_joint;
	cmd_msg.wrist_joint = srv.response.configuration.wrist_joint;

	mJointCommandPublisher.publish(cmd_msg);
}
//TODO make x dependent on z as constraint

/*
 * Keep track of current position in configuration space
 */
void AutonomeArmController::armPositionCB(const arm::armCoordinatesPos &msg)
{
	mCurrentPos = msg;
}

/*
 * Initialize AutonomeArmController node
 */
void AutonomeArmController::init()
{
	mJointCommandPublisher			= mNodeHandle.advertise<arm::armJointPos>("/ArmPositionTopic", 10);	//temporary float64
	mCurrentPositionSubscriber 		= mNodeHandle.subscribe("/armCoordinatePositionFeedbackTopic", 1, &AutonomeArmController::armPositionCB, this);
	mCommandSubscriber 				= mNodeHandle.subscribe("/cmd_position", 1, &AutonomeArmController::cmdCB, this);
	mKinematicsClient 				= mNodeHandle.serviceClient<arm::IK>("IK");
	ROS_INFO("AutonomeArmController initialized");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AutonomeArmController");

	AutonomeArmController armController;
	armController.init();

	ros::spin();

	return EXIT_SUCCESS;	// 0
}
