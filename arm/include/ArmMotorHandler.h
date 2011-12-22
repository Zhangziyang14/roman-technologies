/*
 * ArmMotorHandler.h
 *
 *  Created on: 19 dec. 2011
 *      Author: sjors
 */

#ifndef ARMMOTORHANDLER_H_
#define ARMMOTORHANDLER_H_

#define SHOULDERMOTOR_ENCODER_COUNT		(500)
#define SHOULDERMOTOR_CORRECTION_FACTOR (1.2)	 // encoder multiplication factor to correct 3mxel angle limits

#define SIDEMOTOR_ENCODER_COUNT			(500)
#define SIDEMOTOR_CORRECTION_FACTOR		(1.2)	 // encoder multiplication factor to correct 3mxel angle limits


#define EXT_INIT_MODE_TORQUE (0.001)			 // torque required for ext_init mode
#define EXT_INIT_MODE_ACCEL	(0.5)				 // acceleration for ext_init

#define EXT_INIT_MODE_SHOULDER_SPEED	(-0.5)	 // negative speed for downwards angle initialization
#define EXT_INIT_MODE_SIDEJOINT_SPEED	(-0.2)	 // negative speed for outwards angle initialization
#define DEFAULT_SPEED 					(0.5)
#define DEFAULT_ACCEL					(0.5)

#define SHOULDERMOTOR_MIN_ANGLE 		(0.0)
#define SHOULDERMOTOR_MAX_ANGLE 		(2.25)

#define SIDEJOINT_MIN_ANGLE				(0.0)
#define SIDEJOINT_MAX_ANGLE				(3.0)



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
	void Run();
};




#endif /* ARMMOTORHANDLER_H_ */
