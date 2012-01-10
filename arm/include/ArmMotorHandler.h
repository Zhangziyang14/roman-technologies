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
#define DEFAULT_ACCEL					(0.5)

#define SHOULDERMOTOR_OFFSET			(1.66)
#define SIDEJOINT_OFFSET				(0.53)

#define SHOULDERMOTOR_MIN_ANGLE 		(-1.1)	//Relative to virtual origin
#define SHOULDERMOTOR_MAX_ANGLE 		(2.16 - SHOULDERMOTOR_OFFSET)

#define SIDEJOINT_MIN_ANGLE			    (-0.07 - SIDEJOINT_OFFSET)
#define SIDEJOINT_MAX_ANGLE				(1.55)	//Relative to virtual origin

#define SHOULDERMOTOR_START_POS			(-1.1)	//
#define SIDEJOINT_START_POS				(1.5)	// joint is at this position after being initialized

#include <ros/ros.h>
#include <Motor.h>
#include <std_msgs/Float64.h>
#include "arm/armJointPos.h"
#include <boost/thread.hpp>

class ArmMotorHandler
{

private:
	float angleOffset;

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mShoulderAngleSub;		/// Listen for shoulder angles
	ros::Subscriber mSideJointAngleSub;		/// Listen for sideJoint angles
	ros::Subscriber mArmJointPosSub;		/// Listen for armJointPos messages (containing both angles in one message)

	Motor mShoulderMotor;					/// Motor for left wheel
	Motor mSideMotor;						/// Motor for right wheel

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
	void armPosCB(const arm::armJointPos& msg);

	/** Multithreading callbacks **/
	void Run();

	inline double getShoulderAngle()
	{
		return mShoulderMotor.getAngle() * SHOULDERMOTOR_CORRECTION_FACTOR;
	}

	inline double getSideJointAngle()
	{
		return mSideMotor.getAngle() * SIDEMOTOR_CORRECTION_FACTOR;
	}
};




#endif /* ARMMOTORHANDLER_H_ */
