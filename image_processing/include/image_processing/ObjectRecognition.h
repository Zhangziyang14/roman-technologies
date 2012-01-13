/*
 * ObjectRecognizer.h
 *
 *  Created on: 2011-12-22
 *      Author: wilson
 */

#ifndef OBJECTRECOGNITION_H_
#define OBJECTRECOGNITION_H_

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

class ObjectRecognition
{
protected:
	tabletop_collision_map_processing::TabletopCollisionMapProcessing mCollisionmap;
	tabletop_object_detector::TabletopDetection mObject;

	ros::NodeHandle mNodeHandle;				//ROS node handler
	ros::Subscriber mCommandSubscriber;			//Listens to commands to start image recognition
	ros::Publisher 	mObjectPosePublisher;		//Publishes the pose of the recognised object
	int mObjectToFind;							//ID of the object the robot should look for

public:
	ObjectRecognition(): mNodeHandle(""){}
	~ObjectRecognition()
	{
		mNodeHandle.shutdown();
	}

	void init();

	void recognizeCB(const std_msgs::UInt8 &msg);

	int inline getObjectID()
	{
		return mObjectToFind;
	}

	void inline publishObjectPose(const geometry_msgs::PoseStamped msg)
	{
		mObjectPosePublisher.publish(msg);
	}
};

#endif /* OBJECTRECOGNITION_H_ */
