/*
 * BaseController.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: wilson
 */

#include <BaseController.h>

/**
 * Checks motor connections for availability.
 */
/*void BaseController::checkConnections()
{
		if(mSpeed_sub.getNumPublishers() == 0)
		{
			ROS_INFO("%s has died.", mSpeed_sub.getTopic().c_str());
//			command("shell");
		}
		else if(mKey_sub.getNumPublishers() == 0)
		{
			ROS_INFO("%s has  died",mKey_sub.getTopic().c_str());

		}
}*/

/**
 * Reads the speed feedback from MotorHandler
 */
void BaseController::readCurrentSpeed(const geometry_msgs::Twist& msg)
{
	mCurrentSpeed = msg;
}

/**
 * Initialise the attributes of the controller
 */
void BaseController::init()
{
	//initialise subscribers
	mKey_sub = mNodeHandle.subscribe("joy", 1, &BaseController::keyCB, this);
	mSpeed_sub = mNodeHandle.subscribe("speedFeedbackTopic", 1, &BaseController::readCurrentSpeed, this);

	// initialise publishers
	mMotorControl_pub = mNodeHandle.advertise<mobile_base::BaseMotorControl>("movementTopic", 1);
	mTweak_pub = mNodeHandle.advertise<mobile_base::tweak>("tweakTopic", 10);

	ROS_INFO("BaseController initialised");
}

/**
 * Handles key events from the PS3 controller.
 */
void BaseController::keyCB(const sensor_msgs::Joy& msg)
{
	mobile_base::BaseMotorControl bmc_msg;
	mobile_base::tweak button;

	//No button is pressed, so sum of vector is zero -> stand still
	if (std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0)
		mKeyPressed = PS3_NONE;

	switch (mControlMode)
	{
	case PS3_CONTROL_GAME:
		//Turn at current position
		bmc_msg.twist.linear.x = 0;
		bmc_msg.twist.angular.z = calcRobotAngularSpeed() * float(msg.axes[PS3_AXIS_LEFT_HORIZONTAL]);
		break;
	case PS3_CONTROL_REMOTE_CONTROL:
		bmc_msg.left_motor_speed = MAX_LINEAR_SPEED * msg.axes[PS3_AXIS_LEFT_VERTICAL];
		bmc_msg.right_motor_speed = MAX_LINEAR_SPEED * msg.axes[PS3_AXIS_RIGHT_VERTICAL];
		break;
	default:
		break;
	}
	mMotorControl_pub.publish(bmc_msg);

	bmc_msg.twist.linear.x = 0;
	bmc_msg.twist.angular.z = 0;
	for (size_t i = 0; i < msg.buttons.size(); i++)
	{
		if(msg.buttons[i] == 0)
			continue;

		switch(i)
		{
		case PS3_X:
		case PS3_S:
		{
			if (mControlMode != PS3_CONTROL_GAME)
				break;

			//Accelerate when X button is pressed and reverse when square button is pressed
			float lin_speed = i == PS3_X ? -MAX_LINEAR_SPEED : MAX_LINEAR_SPEED;
			bmc_msg.twist.linear.x = lin_speed * msg.axes[i];
			mMotorControl_pub.publish(bmc_msg);
			break;
		}

			//Brake if O button has been pressed
		case PS3_O:
			bmc_msg.twist.linear.x = 0;
			bmc_msg.twist.angular.z = 0;
			mMotorControl_pub.publish(bmc_msg);
			break;

			//Tweak buttons for PID control
		case PS3_UP:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_UP;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_UP;
			break;

		case PS3_RIGHT:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_RIGHT;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_RIGHT;
			break;

		case PS3_DOWN:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_DOWN;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_DOWN;
			break;

		case PS3_LEFT:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_LEFT;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_LEFT;
			break;

		case PS3_L1:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_L1;
			button.motorID = MID_LEFT;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_L1;
			break;

		case PS3_R1:
			if(mKeyPressed != PS3_NONE)
				break;

			button.data = PS3_R1;
			button.motorID = MID_RIGHT;
			mTweak_pub.publish(button);
			mKeyPressed = PS3_R1;
			break;


		case PS3_SELECT:
			if (mKeyPressed != PS3_NONE)
				break;

			mControlMode = PS3ControlMode((mControlMode + 1) % PS3_CONTROL_TOTAL);
			ROS_INFO("Switched control mode to %s.", mControlMode == PS3_CONTROL_GAME ? "Game" : "Remote Control");
			mKeyPressed = PS3_SELECT;
			break;

		default:
			break;
		}
	}
}

int main(int argc, char **argv)
{
	// init ros and controller
	ros::init(argc, argv, "controller");
	BaseController base_controller;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	base_controller.init();
	ros::spin();

	return 0;
}
