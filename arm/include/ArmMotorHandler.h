/*
 * ArmMotorHandler.h
 *
 *  Created on: 19 dec. 2011
 *      Author: sjors
 */

#ifndef ARMMOTORHANDLER_H_
#define ARMMOTORHANDLER_H_

#include <ros/ros.h>
#include <Motor.h>
#include <std_msgs/Float64.h>



class ArmMotorHandler
{
protected:
	ros::NodeHandle mNodeHandle;			/// ROS node handle

	ros::Subscriber mShoulderAngleSub;			/// Listen for shoulder angles

	Motor mShoulderMotor;					/// Motor for left wheel
	Motor mSideMotor;						/// Motor for right wheel


public:
	/// Constructor
	ArmMotorHandler(char *path); //: mNodeHandle("~"), mShoulderMotor(MID_ARM_SHOULDER), mSideMotor(MID_ARM_SIDEWAYS)

	/// Destructor
	/** Delete motor interface, close serial port, and shut down node handle */
	~ArmMotorHandler()
	{
		mNodeHandle.shutdown();
	}

	/** Subscriber callbacks **/
	void shoulderCB(const std_msgs::Float64& msg);
};




#endif /* ARMMOTORHANDLER_H_ */
