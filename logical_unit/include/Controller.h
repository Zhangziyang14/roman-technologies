/*
 * Controller.h
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <ros/ros.h>

class Controller
{
private:
	ros::NodeHandle mNodeHandle;

	ros::Publisher mBasePublisher;			//Publishes "Twist" messages, but then in position instead of speed
	ros::Publisher mGripperPublisher;
	ros::Publisher mArmPublisher;
	ros::Publisher mHeadPublisher;

	ros::Subscriber mKinectSubscriber;
	ros::Subscriber mAudioSubscriber;
	ros::Subscriber mPathSubscriber;

public:
	Controller() : mNodeHandle(""){}

	virtual ~Controller(){ mNodeHandle.shutdown();}

	void init();
};

#endif /* CONTROLLER_H_ */
