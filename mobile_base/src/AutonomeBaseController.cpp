/*
 * AutonomeBaseController.cpp
 *
 *  Created on: 2011-11-18
 *      Author: wilson
 */
#include "AutonomeBaseController.h"

/**
 * Reads twist messages from cmd_vel to movementTopic
 */
void AutonomeBaseController::twistCB(const geometry_msgs::Twist &msg)
{
	if(!mLock)
		mCommand.twist = msg;

	//Check front when driving in front
	if(msg.linear.x > 0)
		*mDistance = mFront;

	//Check rear when driving backwards
	else if(msg.linear.x < 0)
		*mDistance = mRear;

	//Check sides when turning on own axe
	else
	{
		//Check left side when turning left
		if(msg.angular.z < 0)
			*mDistance = mLeft;

		//Check right side when turning right
		else if(msg.angular.z > 0)
			*mDistance = mRight;
	}
}

/**
 *	Reads distances from ultrasone sensors
 */
void AutonomeBaseController::readDistanceCB(const mobile_base::sensorFeedback &msg)
{
	mFront	 	= msg.frontCenter;
	mFrontLeft 	= msg.frontLeft;
	mFrontRight	= msg.frontRight;

	mRear		= msg.rearCenter;
	mRearLeft	= msg.rearLeft;
	mRearRight	= msg.rearRight;

	mLeft		= msg.left;
	mRight		= msg.right;
}

//FAULTY CODE!
//TODO CORRECT CORRECTCOURSE()

/**
 * Steers the robot in correct course when necessary

void AutonomeBaseController::correctCourse()
{
	if(mLeft <= 30)
	{
		mLock = true;
		nudgeToRight(mTwist.linear.x);
	}
	else if(mRight <= 30)
	{
		mLock = true;
		nudgeToLeft(mTwist.linear.x);
	}
	else if(mLeft <= 30 && mRight <= 30)
	{
		mLock = true;
		moveForward(mTwist.linear.x * 0.25);
	}
	//usleep(1000000);
}*/

/**
 *  Stream commands from navigation stack when in STATE_NEUTRAL
 */
State AutonomeBaseController::neutral()
{
	mLock = false;

	while(!mLock)
	{
		if(*mDistance < 30)
			return STATE_BLOCKED;
		else
		{
			mCommand_pub.publish(mCommand);
		}
		//correctCourse();
	}
	return STATE_NEUTRAL;
}

/**
 * Decide which way to go when in STATE_BLOCKED
 */
State AutonomeBaseController::blocked()
{
	mLock = true;
	stop();

	//Was the robot trying to go forward before being blocked? Then try to drive around the object
	if(mDistance == &mFront)
	{
		if(!leftIsBlocked())
		{
			//turn left until front space is free and right wall is hugged
			while(!rightIsBlocked() && frontIsBlocked())
				turnLeft(STANDARD_SPEED);
			return STATE_HUG_RIGHT_WALL;
		}
		else if(!rightIsBlocked())
		{
			//turn right until front space is free and left wall is hugged
			while(!leftIsBlocked() && frontIsBlocked())
				turnRight(STANDARD_SPEED);
			return STATE_HUG_LEFT_WALL;
		}
		//robot is boxed by walls, so go back where robot came from
		else
			return STATE_TRY_BACK;
	}
	//Was the robot trying to turn left? Then turn to the right, go a bit forward and try again
	else if(mDistance == &mLeft)
	{
		while(leftIsBlocked() && frontIsBlocked())
			turnRight(STANDARD_SPEED);

		moveForward(STANDARD_SPEED);
		usleep(1000000);
		stop();

		return STATE_NEUTRAL;
	}

	//Was the robot trying to turn right? Then turn to the left, go a bit forward and try again
	else if(mDistance == &mRight)
	{
		while(rightIsBlocked() && frontIsBlocked())
			turnLeft(STANDARD_SPEED);

		moveForward(STANDARD_SPEED);
		usleep(1000000);
		stop();

		return STATE_NEUTRAL;
	}
	//Was the robot trying to go backwards?
	else
	{
		if(!leftIsBlocked())
		{
			//turn left until front space is free and left wall is hugged
			while(!leftIsBlocked() && frontIsBlocked())
				turnLeft(STANDARD_SPEED);
			return STATE_HUG_LEFT_WALL;
		}
		else if(!rightIsBlocked())
		{
			//turn right until front space is free and right wall is hugged
			while(!leftIsBlocked() && frontIsBlocked())
				turnRight(STANDARD_SPEED);
			return STATE_HUG_RIGHT_WALL;
		}
		//robot is boxed by walls, so go back where robot came from
		else
			return STATE_TRY_FORWARD;
	}
	return STATE_BLOCKED;		//will not reach this
}

/**
 *	Keep going back until there is space on either side of the robot when in STATE_TRY_BACK
 */
State AutonomeBaseController::tryBack()
{
	while(mLock)
	{
		if(!rearIsBlocked())
		{
			if(leftIsBlocked() && rightIsBlocked())
				moveBackward(STANDARD_SPEED);
			else
			{
				stop();
				return STATE_NEUTRAL;
			}
		}
		else
		{
			stop();
			return STATE_NEUTRAL;
		}
	}
	return STATE_TRY_BACK;			//Will not reach this
}

/**
 *	Keep going forward until there is space on either side of the robot when in STATE_TRY_FORWARD
 */
State AutonomeBaseController::tryForward()
{
	while(mLock)
	{
		if(!frontIsBlocked())
		{
			if(leftIsBlocked() && rightIsBlocked())
				moveForward(STANDARD_SPEED);
			else
			{
				stop();
				return STATE_NEUTRAL;
			}
		}
		else
		{
			stop();
			return STATE_NEUTRAL;
		}
	}
	return STATE_TRY_BACK;			//Will not reach this
}

/**
 *	Hug left wall until obstacle is avoided
 */
State AutonomeBaseController::hugLeft()
{
	while(mLock)
	{
		if(!frontIsBlocked() && leftIsBlocked())
			moveForward(STANDARD_SPEED);
		else if(!leftIsBlocked())
		{
			stop();
			//turn left to face target again?
			return STATE_NEUTRAL;
		}
		//Object avoidance failed, try to backtrack other solution
		else if(frontIsBlocked() && leftIsBlocked())
		{
			stop();
			return STATE_BACKTRACK;
		}
	}
	return STATE_HUG_LEFT_WALL;		//Will not reach this
}

/**
 *	Hug right wall until obstacle is avoided
 */
State AutonomeBaseController::hugRight()
{
	while(mLock)
	{
		if(!frontIsBlocked() && rightIsBlocked())
			moveForward(STANDARD_SPEED);
		else if(!rightIsBlocked() || (frontIsBlocked() && rightIsBlocked()))
		{
			stop();
			//turn left to face target again?
			return STATE_NEUTRAL;
		}
	}
	return STATE_HUG_RIGHT_WALL;		//Will not reach this
}

/**
 * Go back and try to find another solution
 */
State AutonomeBaseController::backtrack()
{
	while(mLock)
	{
		if(!rearIsBlocked() && rightIsBlocked())
			moveBackward(STANDARD_SPEED);
		//If solution is found or if there is no solution, then give back control to navigation stack
		else if(rearIsBlocked()|| !rightIsBlocked())
		{
			stop();
			return STATE_NEUTRAL;
		}
	}
	return STATE_BACKTRACK;		//will not reach this
}

/**
 * Initialises AutonomeBaseController
 */
void AutonomeBaseController::init()
{
	//initialise publishers
	mAutoCommand_pub 	= mNodeHandle.advertise<mobile_base::AutoMotorControl>("/autoMovementTopic", 1);
	mCommand_pub 		= mNodeHandle.advertise<mobile_base::BaseMotorControl>("/movementTopic", 10);
	mSensorActivate_pub = mNodeHandle.advertise<mobile_base::activateSensors>("/sensorActivateTopic", 1);

	//initialise subscribers
	mNavigation_sub 	= mNodeHandle.subscribe("/cmd_vel", 10, &AutonomeBaseController::twistCB, this);
	mUltrasone_sub 		= mNodeHandle.subscribe("/sensorFeedbackTopic", 10, &AutonomeBaseController::readDistanceCB, this);

	//Remove this from here?
	mobile_base::activateSensors activate_msg;
	activate_msg.stateMask = ULTRASONE_ALL;

	//activate sensors
	mSensorActivate_pub.publish(activate_msg);

	mLock = false;

	ROS_INFO("AutonomeBaseController initialised");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AutonomeBaseController");

	AutonomeBaseController autonomeBaseController;

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	autonomeBaseController.init();

	while(ros::ok)
	{
		/*
		 * TEST 1: forward
		   moveForward();
		 */

		/*
		 * TEST 2: backwards
		   moveBackward(STANDARD_SPEED);
		 */

		/*
		 * TEST 3: turn left
		   turnLeft(STANDARD_SPEED);
		 */

		/*
		 * TEST 4: turn right
		   turnRight(STANDARD_SPEED);
		 */

		/*
		 * TEST 5: nudge left forward
		   moveForward();
		   nudgeToLeft(STANDARD_SPEED);
		   moveForward(STANDARD_SPEED);
		 */

		/*
		 * TEST 6: nudge left backwards
		   moveBackward(STANDARD_SPEED);
		   nudgeToLeft(STANDARD_SPEED);
		   moveBackward(STANDARD_SPEED);
		 */

		/*
		 * TEST 7: nudge right forward
		   moveForward(STANDARD_SPEED);
		   nudgeToRight(STANDARD_SPEED);
		   moveForward(STANDARD_SPEED);
		 */

		/*
		 * TEST 8: nudge right backwards
		   moveBackward(STANDARD_SPEED);
		   nudgeToRight(STANDARD_SPEED);
		   moveBackward(STANDARD_SPEED);
		*/

		/*
		 * TEST 9: neutral
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			autonomeBaseController.setCurrentState(autonomeBaseController.neutral());
			break;
		 case STATE_BLOCKED:
			autonomeBaseController.setCurrentState(autonomeBaseController.blocked());
			break;
		 case STATE_HUG_RIGHT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugRight());
			break;
		 case STATE_HUG_LEFT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugLeft());
			break;
		 case STATE_TRY_BACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryBack());
			break;
		 case STATE_TRY_FORWARD:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryForward());
			break;
		 case STATE_BACKTRACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.backtrack());
			break;
		}*/

		/*
		 * TEST 10: blocked
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			autonomeBaseController.setCurrentState(autonomeBaseController.blocked());
			break;
		 case STATE_HUG_RIGHT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_LEFT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_BACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_FORWARD:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BACKTRACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		}
		 */

		/*
		 * TEST 11: hug_right_wall
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_RIGHT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugRight());
			break;
		 case STATE_HUG_LEFT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_BACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_FORWARD:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BACKTRACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		}
		*/

		/*
		 * TEST 12:hug_left_wall
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_RIGHT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_LEFT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugLeft());
			break;
		 case STATE_TRY_BACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_FORWARD:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BACKTRACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		}
		*/

		/*
		 * TEST 13:tryBack
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_RIGHT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_LEFT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_BACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryLeft());
			break;
		 case STATE_TRY_FORWARD:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BACKTRACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		}
		*/

		/*
		 * TEST 14:tryForward
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_RIGHT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_LEFT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_BACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_FORWARD:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryForward());
			break;
		 case STATE_BACKTRACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		}
		*/

		/*
		 * TEST 15:backtrack
		 *
		 switch(autonomeBaseController.getCurrentState())
		 {
		 case STATE_NEUTRAL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BLOCKED:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_RIGHT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_HUG_LEFT_WALL:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_BACK:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_TRY_FORWARD:
			cout << autonomeBaseController.getCurrentState() << endl;
			break;
		 case STATE_BACKTRACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.backtrack());
			break;
		}
		*/

		/*
		 * FINAL TEST: Everything
		 *
		//State diagram
		switch(autonomeBaseController.getCurrentState())
		{
		case STATE_NEUTRAL:
			autonomeBaseController.setCurrentState(autonomeBaseController.neutral());
			break;
		case STATE_BLOCKED:
			autonomeBaseController.setCurrentState(autonomeBaseController.blocked());
			break;
		case STATE_HUG_RIGHT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugRight());
			break;
		case STATE_HUG_LEFT_WALL:
			autonomeBaseController.setCurrentState(autonomeBaseController.hugLeft());
			break;
		case STATE_TRY_BACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryBack());
			break;
		case STATE_TRY_FORWARD:
			autonomeBaseController.setCurrentState(autonomeBaseController.tryForward());
			break;
		case STATE_BACKTRACK:
			autonomeBaseController.setCurrentState(autonomeBaseController.backtrack());
			break;
		}
		*/
		ros::spinOnce();
	}
	return 0;
}
