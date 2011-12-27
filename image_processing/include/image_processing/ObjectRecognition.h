/*
 * ObjectRecognizer.h
 *
 *  Created on: 2011-12-22
 *      Author: wilson
 */

#ifndef OBJECTRECOGNITION_H_
#define OBJECTRECOGNITION_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

class ObjectRecognition
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	ros::Subscriber mCommandSubscriber;			//Listens to commands to start image recognition
	tabletop_collision_map_processing::TabletopCollisionMapProcessing mCollisionmap;
	tabletop_object_detector::TabletopDetection mObject;

public:
	ObjectRecognition(): mNodeHandle(""){}
	~ObjectRecognition()
	{
		mNodeHandle.shutdown();
	}

	void init();

	void recognizeCB(const std_msgs::Bool &msg);
};

#endif /* OBJECTRECOGNITION_H_ */
