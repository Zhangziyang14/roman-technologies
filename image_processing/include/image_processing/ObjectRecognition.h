/*
 * ObjectRecognizer.h
 *
 *  Created on: 2011-12-22
 *      Author: wilson
 */

#ifndef OBJECTRECOGNITION_H_
#define OBJECTRECOGNITION_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>


// REMOVE THIS
#include <tf/transform_listener.h>

#define VIEW_OBJECTS_ANGLE	0.4	

class ObjectRecognition
{
protected:
	tabletop_collision_map_processing::TabletopCollisionMapProcessing mCollisionmap;
	tabletop_object_detector::TabletopDetection mObject;

	ros::NodeHandle mNodeHandle;				//ROS node handler
	ros::Subscriber mCommandSubscriber;			//Listens to commands to start image recognition
	ros::Publisher 	mObjectPosePublisher;		//Publishes the pose of the recognised object
	int mObjectToFind;							//ID of the object the robot should look for
	double mMinQuality;							//Threshold for when a model is considered recognised
	tf::TransformListener mTransformListener;

public:
	ObjectRecognition();
	~ObjectRecognition()
	{
		mNodeHandle.shutdown();
	}

	void recognizeCB(const std_msgs::Int32 &msg);

	int inline getObjectID()
	{
		return mObjectToFind;
	}

	void inline publishObjectPose(geometry_msgs::PoseStamped msg)
	{
		geometry_msgs::PoseStamped msgToArm;
		mTransformListener.transformPose("arm_frame", msg, msgToArm);

		ROS_INFO("Before: [%lf, %lf, %lf]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
		ROS_INFO("After: [%lf, %lf, %lf]", msgToArm.pose.position.x, msgToArm.pose.position.y, msgToArm.pose.position.z);

		mObjectPosePublisher.publish(msgToArm);
	}

	void spin();
};

#endif /* OBJECTRECOGNITION_H_ */
