/*
 * ArmMotorHandler.h
 *
 *  Created on: 19 dec. 2011
 *      Author: sjors
 */

#ifndef ARMMOTORHANDLER_H_
#define ARMMOTORHANDLER_H_

#define SHOULDERMOTOR_ENCODER_COUNT		(500)
#define SHOULDERMOTOR_CORRECTION_FACTOR (2.2)	 // encoder multiplication factor to correct 3mxel angle limits

#define SIDEMOTOR_ENCODER_COUNT			(500)
#define SIDEMOTOR_CORRECTION_FACTOR		(1.5)	 // encoder multiplication factor to correct 3mxel angle limits
#define SAFETY_OFFSET					0.05	 // Safety offset for overshoots during intialising

#define EXT_INIT_MODE_TORQUE (0.001)			 // torque required for ext_init mode
#define EXT_INIT_MODE_ACCEL	(0.5)				 // acceleration for ext_init

#define EXT_INIT_MODE_SHOULDER_SPEED	(-0.4)	 // negative speed for downwards angle initialization
#define EXT_INIT_MODE_SIDEJOINT_SPEED	(-0.4)	 // negative speed for outwards angle initialization
#define DEFAULT_SPEED 					(0.5)
#define DEFAULT_ACCEL					(1.0)

#define SHOULDERMOTOR_OFFSET			(1.66)
#define SIDEJOINT_OFFSET				(0.48)

#define SHOULDERMOTOR_MIN_ANGLE 		(-1.1)	//Relative to virtual origin
#define SHOULDERMOTOR_MAX_ANGLE 		(1.96 - SHOULDERMOTOR_OFFSET)

#define SIDEJOINT_MIN_ANGLE			    (0.14 - SIDEJOINT_OFFSET)
#define SIDEJOINT_MAX_ANGLE				(1.55)	//Relative to virtual origin

#define SHOULDERMOTOR_START_POS			(-1.1)	//
#define SIDEJOINT_START_POS				(1.5)	// joint is at this position after being initialized

#define SHOULDER_SAFETY_TRESHOLD		0.05
#define SIDE_SAFETY_TRESHOLD			0.05

#include <ros/ros.h>
#include <motors/Motor.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "nero_msgs/ArmJoint.h"

class ArmMotorHandler
{

private:
	float angleOffset;

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Publisher mArmPosFeedbackPub;		/// Publishes current position of the arm in joint space
	ros::Publisher mArmSpeedFeedbackPub;	/// Publishes current speeds of the arm

	ros::Subscriber mShoulderAngleSub;		/// Listens to shoulder angles
	ros::Subscriber mSideJointAngleSub;		/// Listens to sideJoint angles
	ros::Subscriber mArmJointPosSub;		/// Listens to armJointPos messages (containing both angles in one message)
	ros::Subscriber mStopSubscriber;		/// Listens to stop commands from Controller
	ros::Subscriber	mArmJointSpeedSub;		/// Listens to speed commands for the arm

	Motor mShoulderMotor;					/// Motor for left wheel
	Motor mSideMotor;						/// Motor for right wheel

	double mCurrentShoulderJointPos, mCurrentSideJointPos;		/// Keeps track of the current joint positions of both motors
	double mCurrentShoulderJointSpeed, mCurrentSideJointSpeed;	/// Keeps track of the current speeds of both motors

	// initialize motor: find reference point
	bool init();

	// set angles
	void setShoulderAngle(double angle);
	void setSideJointAngle(double angle);


public:
	/// Constructor
	ArmMotorHandler(char *path);


	/// Destructor
	/** shut down node handle, stop 'run' thread */
	~ArmMotorHandler()
	{
		mNodeHandle.shutdown();
	}


	/** Subscriber callbacks **/
	void shoulderCB(const std_msgs::Float64& msg);
	void sideJointCB(const std_msgs::Float64& msg);
	void armPosCB(const nero_msgs::ArmJoint& msg);
	void stopCB(const std_msgs::Bool& msg);
	void speedCB(const nero_msgs::ArmJoint& msg);
	void publishArmPosition();
	void publishArmSpeed();

	/** Multi threading callbacks **/
	//void Run();

	inline double getShoulderAngle()
	{
		return mShoulderMotor.getAngle() * SHOULDERMOTOR_CORRECTION_FACTOR - SHOULDERMOTOR_OFFSET;
	}

	inline double getSideJointAngle()
	{
		return mSideMotor.getAngle() * SIDEMOTOR_CORRECTION_FACTOR - SIDEJOINT_OFFSET;
	}

	inline double getShoulderSpeed()
	{
		return mShoulderMotor.getRotationSpeed();
	}

	inline double getSideJointSpeed()
	{
		return mSideMotor.getRotationSpeed();
	}

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* ARMMOTORHANDLER_H_ */
