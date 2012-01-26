#include "AutonomeArmController.h"
/*
 *	Listens to commands and translates it to joint commands for ArmMotorHandler using inverse kinematics
 */
void AutonomeArmController::cmdCB(const geometry_msgs::Pose &msg)
{
	arm::IK srv;

	//Listen to command
	srv.request.target = msg;

	//Check constraints
	if(msg.position.z > MAX_Z_VALUE)
	{
		ROS_WARN("REACHED MAX_Z_VALUE");
		srv.request.target.position.z = MAX_Z_VALUE;
	}
	else if(msg.position.z  < MIN_Z_VALUE)
	{
		ROS_WARN("REACHED MAX_Z_VALUE");
		srv.request.target.position.z = MIN_Z_VALUE;
	}

	if(msg.position.x > MAX_X_VALUE)
	{
		ROS_WARN("REACHED MAX_X_VALUE");
		srv.request.target.position.x = MAX_X_VALUE;
	}
	else if(msg.position.x < MIN_X_VALUE)
	{
		ROS_WARN("REACHED MIN_X_VALUE");
		srv.request.target.position.x = MIN_X_VALUE;
	}

	//Calculate joint configurations with IK solver
	if((srv.request.target.position.z <= MAX_Z_VALUE) && (srv.request.target.position.z >= MIN_Z_VALUE)
			&& (srv.request.target.position.x <= MAX_X_VALUE) && (srv.request.target.position.x <= MAX_X_VALUE))
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

/*
 * Keep track of current position in configuration space

void AutonomeArmController::armPositionFeedbackCB(const geometry_msgs::Pose &msg)
{
	mCurrentPos = msg;
}*/

/*
 * Initialize AutonomeArmController node
 */
void AutonomeArmController::init()
{
	mJointCommandPublisher			= mNodeHandle.advertise<arm::armJointPos>("/armJointPositionTopic", 10);
	//mCurrentPositionSubscriber 	= mNodeHandle.subscribe("/armCoordinatePositionFeedbackTopic", 1, &AutonomeArmController::armPositionFeedbackCB, this);
	mCommandSubscriber 				= mNodeHandle.subscribe("/cmd_arm_position", 1, &AutonomeArmController::cmdCB, this);
	mKinematicsClient 				= mNodeHandle.serviceClient<arm::IK>("IK");
	ROS_INFO("AutonomeArmController initialised");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AutonomeArmController");

	AutonomeArmController armController;
	armController.init();

	int sleep_rate;
	armController.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		sleep.sleep();
		ros::spinOnce();
	}

	return EXIT_SUCCESS;	// 0
}
