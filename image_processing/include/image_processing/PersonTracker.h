/*
 * PersonTracker.h
 *
 *  Created on: May 22, 2012
 *      Author: hans
 */

#ifndef PERSONTRACKER_H_
#define PERSONTRACKER_H_

#include "ros/ros.h"
#include <queue>

#include "image_server/OpenCVTools.h"
#include "nero_msgs/ColorDepth.h"
#include "nero_msgs/QueryCloud.h"

#include "geometry_msgs/PointStamped.h"

#include "tf/transform_listener.h"

#define DEPTH_TOLERANCE 20 // in mm
#define ABSOLUTE_DEPTH_TOLERANCE 200 // in mm

class PersonTracker
{
private:
	ros::NodeHandle mNodeHandle;
	tf::TransformListener mTransformListener;

	ros::Subscriber mColorDepthImageSub;
	ros::Publisher mTrackedPointPub;
	ros::ServiceClient mProjectClient;

	bool mShowOutput;
	bool mTracking;
	bool mWaitForBBox;
	cv::Mat mLastImage;
	cv::Mat mLastDepthImage;
	cv::Scalar yellow, blue;

	cv::Rect mBBox;
	cv::Point2i mCOG;
	uint16_t mLastDepth;
	uint16_t mCurrDepth;
	cv::Point2f mLastLoc;
	cv::Point2f mCurrLoc;

public:
	PersonTracker();

	void imageColorDepthCb(const nero_msgs::ColorDepthPtr &image);

	bool seedImage(cv::Mat depth, cv::Mat &result, cv::Point2i seed, cv::Rect &r, cv::Point2i &cog);
	geometry_msgs::PointStamped getWorldPoint(geometry_msgs::Point p);

	void spin();
};

#endif /* PERSONTRACKER_H_ */
