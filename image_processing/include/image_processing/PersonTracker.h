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
#include "nero_msgs/SetActive.h"

#include "geometry_msgs/PointStamped.h"

#include "tf/transform_listener.h"

#define DEPTH_TOLERANCE 20 // in mm
#define ABSOLUTE_DEPTH_TOLERANCE 200 // in mm

#define HIGH_POINT_OFFSET -0.1

class PersonTracker
{
private:
	ros::NodeHandle mNodeHandle;
	tf::TransformListener mTransformListener;

	ros::Subscriber mInitialPointSub;
	ros::Subscriber mColorDepthImageSub;
	ros::Publisher mTrackedPointPub;
	ros::Publisher mHighPointPub;
	ros::ServiceClient mProjectClient;
	ros::ServiceClient mFocusFaceClient;

	bool mShowOutput;
	bool mTracking;
	bool mWaitForBBox;
	cv::Mat mLastImage;
	cv::Mat mLastDepthImage;
	cv::Scalar yellow, blue;

	cv::Rect mBBox;
	cv::Point2i mCOG;
	uint16_t mLastDepth;

public:
	PersonTracker();

	void imageColorDepthCb(const nero_msgs::ColorDepthPtr &image);

	void initialPointCb(const geometry_msgs::Point &point);

	bool seedImage(cv::Mat depth, cv::Mat &result, cv::Point2i seed, cv::Rect &r, cv::Point2i &cog, cv::Point2i &highPoint);
	geometry_msgs::PointStamped getWorldPoint(cv::Point2i p, const char *frame = "/base_link");

	inline void setFocusFace(bool active) { nero_msgs::SetActive srv;  srv.request.active = active; mFocusFaceClient.call(srv); };

	void spin();
};

#endif /* PERSONTRACKER_H_ */
