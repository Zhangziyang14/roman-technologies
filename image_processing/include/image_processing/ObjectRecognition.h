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

#include "nero_msgs/FindObject.h"
#include "nero_msgs/SetActive.h"

// REMOVE THIS
#include <tf/transform_listener.h>

class ObjectRecognition
{
protected:
	tabletop_object_detector::TabletopDetection mObject;

	ros::NodeHandle mNodeHandle;				//ROS node handler

	ros::ServiceClient mObjectRecognizeClient;		//Service client for tabletop_object_detection
	ros::ServiceClient mSendCloudsClient;		//Service client for cloud control
	ros::ServiceServer mFindObjectServer;		/// Server for finding an object

	double mMinQuality;							//Threshold for when a model is considered recognised
	tf::TransformListener mTransformListener;

public:
	ObjectRecognition();
	~ObjectRecognition()
	{
		mNodeHandle.shutdown();
	}

	bool recognizeCB(nero_msgs::FindObject::Request &req, nero_msgs::FindObject::Response &res);

	void spin();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };

	inline void sendClouds(bool active) { nero_msgs::SetActive srv; srv.request.active = active; mSendCloudsClient.call(srv); };
};

#endif /* OBJECTRECOGNITION_H_ */
