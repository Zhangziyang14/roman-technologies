/*
 * AutonomeBaseController.h
 *
 *  Created on: 2011-11-18
 *      Author: wilson
 */

#ifndef AUTONOMEBASECONTROLLER_H_
#define AUTONOMEBASECONTROLLER_H_

#include <ros/ros.h>
#include <mobile_base/sensorFeedback.h>
#include <mobile_base/activateSensors.h>
#include <mobile_base/BaseMotorControl.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

#define STANDARD_SPEED 						0.5			// [m/s]
#define STANDARD_ANGULAR_SPEED 				0.5			// [m/s]
#define OBJECT_AVOIDANCE_DISTANCE			30			// [cm]

enum UltrasoneActivate
{
	NOTHING,
	ULTRASONE_FRONT,
	ULTRASONE_REAR,
	ULTRASONE_ALL,
};

enum State
{
	STATE_NEUTRAL,
	STATE_BLOCKED,
	STATE_HUG_RIGHT_WALL,
	STATE_HUG_LEFT_WALL,
	STATE_TRY_BACK,
	STATE_TRY_FORWARD,
	STATE_BACKTRACK,
};

class AutonomeBaseController
{
private:
	ros::NodeHandle mNodeHandle;			/// ROS node handle
	ros::Publisher mAutoCommand_pub;		/// Publishes auto movement commands
	ros::Publisher mCommand_pub;				/// Publishes twist messages to movementTopic
	ros::Publisher mSensorActivate_pub;		/// Activates ultrasone sensors on arduino

	ros::Subscriber mNavigation_sub;		/// Listens to twist messages from ros navigation stack
	ros::Subscriber mUltrasone_sub;			/// Listens to distances from ultrasone sensors
	geometry_msgs::Twist mTwist; 			/// Twist message to be published over movementTopic
	mobile_base::BaseMotorControl mCommand;	/// Motor command containing twist message

	State mCurrentState;					/// Keeps track of the current state for reactive object avoidance
	double *mDistance;						/// Distance to object (if any) in the direction of the robot's movement
	bool mLock;								/// Blocks any publishes to movementTopic except for course correction

	double mFront, mFrontLeft, mFrontRight, mLeft, mRight, mRear, mRearLeft, mRearRight;

public:
	AutonomeBaseController() : mNodeHandle(""){};

	~AutonomeBaseController()
	{
		mNodeHandle.shutdown();
	}

	//void correctCourse();
	void readDistanceCB(const mobile_base::sensorFeedback &msg);
	void twistCB(const geometry_msgs::Twist &msg);
	void init();

	State neutral();
	State blocked();
	State hugRight();
	State hugLeft();
	State tryBack();
	State tryForward();
	State backtrack();

	inline State getCurrentState(){ return mCurrentState; };
	inline void setCurrentState(State state){ mCurrentState = state; };

	/**
	 * Various checks for blocks while driving
	 */

	//is front blocked?
	inline bool frontIsBlocked(){ return (mFront < OBJECT_AVOIDANCE_DISTANCE) && (mFrontLeft < OBJECT_AVOIDANCE_DISTANCE) && (mFrontRight < OBJECT_AVOIDANCE_DISTANCE); };

	//is rear blocked?
	inline bool rearIsBlocked(){ return (mRear < OBJECT_AVOIDANCE_DISTANCE) && (mRearLeft < OBJECT_AVOIDANCE_DISTANCE) && (mRearRight < OBJECT_AVOIDANCE_DISTANCE); };

	//is left side blocked?
	inline bool leftIsBlocked(){ return mLeft < OBJECT_AVOIDANCE_DISTANCE; };

	//is right side blocked?
	inline bool rightIsBlocked(){ return mRight < OBJECT_AVOIDANCE_DISTANCE; };

	//is the robot in a corner?
	inline bool isBoxed(){ return leftIsBlocked() && rightIsBlocked() && frontIsBlocked(); };


	/**
	 * Movement for reactive object avoidance
	 */

	//Nudge the robot a bit to the left
	//standard speed or current speed?
	inline void nudgeToLeft(double speed){ mCommand.left_motor_speed = 0.75*speed; mCommand.right_motor_speed = speed; mCommand_pub.publish(mCommand);};

	//Nudge the robot a bit to the right
	//standard speed or current speed?
	inline void nudgeToRight(double speed){mCommand.left_motor_speed = speed; mCommand.right_motor_speed = 0.75*speed; mCommand_pub.publish(mCommand);};

	//Moves the robot forward
	inline void moveForward(double speed){ mCommand.left_motor_speed = speed; mCommand.right_motor_speed = speed; mCommand_pub.publish(mCommand);};

	//Moves the robot backwards
	inline void moveBackward(double speed){ mCommand.left_motor_speed = -speed; mCommand.right_motor_speed = -speed; mCommand_pub.publish(mCommand);};

	//Stops the robot
	inline void stop(){ mCommand.left_motor_speed = 0; mCommand.right_motor_speed = 0; mCommand_pub.publish(mCommand); };

	//Turns the robot to the left
	inline void turnLeft(double speed){ stop(); mCommand.left_motor_speed = -speed; mCommand.right_motor_speed = speed; mCommand_pub.publish(mCommand); };

	//Turns the robot to the right
	inline void turnRight(double speed){ stop(); mCommand.left_motor_speed = speed; mCommand.right_motor_speed = -speed; mCommand_pub.publish(mCommand); };



};

#endif /* AUTONOMEBASECONTROLLER_H_ */
