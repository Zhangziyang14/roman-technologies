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
#include "nero_msgs/PitchYaw.h"

#include "geometry_msgs/PointStamped.h"

#include "tf/transform_listener.h"

#define DEPTH_TOLERANCE 20 // in mm
#define ABSOLUTE_DEPTH_TOLERANCE 200 // in mm

#define HIGH_POINT_OFFSET -0.2

#define STOP_SPEED_TOLERANCE 0.005

class PersonTracker
{
private:
	ros::NodeHandle mNodeHandle;
	tf::TransformListener mTransformListener;

	ros::Subscriber mInitialPointSub;
	ros::Subscriber mColorDepthImageSub;
	ros::Subscriber mHeadSpeedSub;
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

	nero_msgs::PitchYaw mHeadSpeed;

	inline bool canMoveHead() { return mHeadSpeed.pitch < STOP_SPEED_TOLERANCE && mHeadSpeed.yaw < STOP_SPEED_TOLERANCE; };

public:
	PersonTracker();

	void headSpeedCb(const nero_msgs::PitchYaw &msg);
	void imageColorDepthCb(const nero_msgs::ColorDepthPtr &image);
	void initialPointCb(const geometry_msgs::Point &point);

	bool seedImage(cv::Mat depth, cv::Mat &result, cv::Point2i seed, cv::Rect &r, cv::Point2i &cog, cv::Point2i &highPoint);
	geometry_msgs::PointStamped getWorldPoint(cv::Point2i p, const char *frame = "/base_link", uint16_t depth = 0);

	inline void setFocusFace(bool active) { nero_msgs::SetActive srv;  srv.request.active = active; mFocusFaceClient.call(srv); };

	void sendHeadPosition(cv::Point2i point);

	void spin();
};

#endif /* PERSONTRACKER_H_ */
