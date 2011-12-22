/*
 * ArmMotorHandler.h
 *
 *  Created on: 19 dec. 2011
 *      Author: sjors
 */

#ifndef ARMMOTORHANDLER_H_
#define ARMMOTORHANDLER_H_

#define SHOULDERMOTOR_CORRECTION_FACTOR (0.5625) // old [DPR2] gear ratio: 27/48
#define SHOULDERMOTOR_TRANSMISSION_RATIO (3.0)	 // current gear ratio

#define EXT_INIT_MODE_TORQUE (0.001)			 // torque required for ext_init mode
#define EXT_INIT_MODE_ACCEL	(0.5)				 // acceleration for ext_init
#define EXT_INIT_MODE_SPEED	(-0.5)				 // negative speed for downwards angle initialization

#define SHOULDERMOTOR_MIN_ANGLE (0.0)
#define SHOULDERMOTOR_MAX_ANGLE (2.25)



#include <ros/ros.h>
#include <Motor.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>

class ArmMotorHandler
{

private:
	float angleOffset;

protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mShoulderAngleSub;			/// Listen for shoulder angles

	Motor mShoulderMotor;					/// Motor for left wheel
	Motor mSideMotor;						/// Motor for right wheel

	// initialize motor: find reference point
	bool init();


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
	void Run();
};




#endif /* ARMMOTORHANDLER_H_ */
