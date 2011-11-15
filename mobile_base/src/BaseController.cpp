/*
 * BaseController.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: wilson
 */

#include <BaseController.h>

/**
 * Reads the speed feedback from MotorHandler
 */
void BaseController::readCurrentSpeed(const geometry_msgs::Twist &msg)
{
	mCurrentSpeed = msg;
	std_msgs::UInt8 ultrasone_msg;

	if(msg.linear.x > 0)
		ultrasone_msg.data = ULTRASONE_FRONT;

	if(msg.linear.x < 0)
		ultrasone_msg.data = ULTRASONE_REAR;

	if(msg.angular.z || msg.linear.x == 0)
			ultrasone_msg.data = ULTRASONE_ALL;

		mUltrasone_pub.publish(ultrasone_msg);
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
	mUltrasone_pub = mNodeHandle.advertise<std_msgs::UInt8>("/sensorActivateTopic", 10);

	ROS_INFO("BaseController initialised");
}

/**
 * Handles key events from the PS3 controller.
 */
void BaseController::keyCB(const sensor_msgs::Joy& msg)
{
	// make messages
	mobile_base::BaseMotorControl bmc_msg;
	mobile_base::tweak tweak_msg;

	// initialise values (or are they by default 0?)
	bmc_msg.twist.linear.x = 0;
	bmc_msg.twist.angular.z = 0;
	bmc_msg.left_motor_speed = 0.f;
	bmc_msg.right_motor_speed = 0.f;

	// initialise tweak values
	tweak_msg.scaleUp = false;
	tweak_msg.scaleDown = false;
	tweak_msg.toggleForward = false;
	tweak_msg.toggleBackward = false;
	tweak_msg.motorID = MID_NONE;

	// did we need to send a message?
	bool sendMsg = false;

	//No button is pressed, so sum of vector is zero
	if (std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0)
	{
		// stop after releasing a key
		if (mKeyPressed != PS3_NONE)
			sendMsg = true;
		mKeyPressed = PS3_NONE;
	}

	switch (mControlMode)
	{
	case PS3_CONTROL_GAME:
		// only send stop messages once
		if (mPrevAngular == 0.f && msg.axes[PS3_AXIS_LEFT_HORIZONTAL] == 0.f)
			break;

		//Turn at current position
		bmc_msg.twist.angular.z = calcRobotAngularSpeed() * msg.axes[PS3_AXIS_LEFT_HORIZONTAL];
		mPrevAngular = bmc_msg.twist.angular.z;
		sendMsg = true;
		break;
	case PS3_CONTROL_REMOTE_CONTROL:
		// only send stop messages once
		if (mPrevLeftSpeed == 0.f && mPrevRightSpeed == 0.f && msg.axes[PS3_AXIS_LEFT_VERTICAL] == 0.f &&
			msg.axes[PS3_AXIS_RIGHT_VERTICAL])
			break;

		// scale between 0 and MAX_LINEAR_SPEED.
		bmc_msg.left_motor_speed = MAX_LINEAR_SPEED * msg.axes[PS3_AXIS_LEFT_VERTICAL];
		bmc_msg.right_motor_speed = MAX_LINEAR_SPEED * msg.axes[PS3_AXIS_RIGHT_VERTICAL];
		mPrevLeftSpeed = bmc_msg.left_motor_speed;
		mPrevRightSpeed = bmc_msg.right_motor_speed;
		sendMsg = true;
		break;
	default:
		break;
	}

	// iterate all buttons
	for (size_t i = 0; i < msg.buttons.size(); i++)
	{
		// is this button pressed?
		if(msg.buttons[i] == 0)
			continue;

		switch(i)
		{
		case PS3_X: // forward motion
		case PS3_S: // backward motion
		{
			if (mControlMode != PS3_CONTROL_GAME)
				break;

			//Accelerate when X button is pressed and reverse when square button is pressed
			float lin_speed = i == PS3_X ? -MAX_LINEAR_SPEED : MAX_LINEAR_SPEED;
			bmc_msg.twist.linear.x = lin_speed * msg.axes[i];	


			sendMsg = true;
			break;
		}

		//Brake if O button has been pressed
		case PS3_O:
			bmc_msg.twist.linear.x = 0;
			bmc_msg.twist.angular.z = 0;
			sendMsg = true;
			break;

		// scale up/down current P-I-D value
		case PS3_UP:
		case PS3_DOWN:
			if(mKeyPressed != PS3_NONE)
				break;

			if (i == PS3_UP)
				tweak_msg.scaleUp = true;
			if (i == PS3_DOWN)
				tweak_msg.scaleDown = true;
			mTweak_pub.publish(tweak_msg);
			mKeyPressed = PS3Key(i);
			break;

		// toggle through P-I-D focus
		case PS3_RIGHT:
		case PS3_LEFT:
			if(mKeyPressed != PS3_NONE)
				break;

			if (i == PS3_LEFT)
				tweak_msg.toggleForward = true;
			if (i == PS3_RIGHT)
				tweak_msg.toggleBackward = true;
			mTweak_pub.publish(tweak_msg);
			mKeyPressed = PS3Key(i);
			break;

		// select a motor
		case PS3_L1:
		case PS3_R1:
			if(mKeyPressed != PS3_NONE)
				break;

			tweak_msg.motorID = i == PS3_L1 ? MID_LEFT : MID_RIGHT;
			mTweak_pub.publish(tweak_msg);
			mKeyPressed = PS3Key(i);
			break;

		// toggle control mode
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

	if (sendMsg)
		mMotorControl_pub.publish(bmc_msg);
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
