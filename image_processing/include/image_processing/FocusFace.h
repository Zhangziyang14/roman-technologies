/*
 * FocusFace.h
 *
 *  Created on: Feb 7, 2012
 *      Author: hans
 */

#ifndef FOCUSFACE_H_
#define FOCUSFACE_H_

#define WEBCAM

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "nero_msgs/PitchYaw.h"
#include "nero_msgs/SetActive.h"
#include "nero_msgs/QueryCloud.h"

#include "image_server/OpenCVTools.h"

#include "tf/transform_listener.h"

#define STOP_SPEED_TOLERANCE 0.005
#define MAX_CORNERS 200

#define MIN_FEATURE_COUNT 10
#define OUTLIER_THRESHOLD 3.0
#define MAX_TRACKING_TIME 3.0

class FocusFace
{
private:
	ros::NodeHandle mNodeHandle;
	image_transport::ImageTransport mImageTransport;

	cv::CascadeClassifier mFrontalFaceCascade;
	cv::CascadeClassifier mProfileCascade;
	cv::CascadeClassifier mFrontalFace2Cascade;
	cv::Mat mPrevFrame;
	std::vector<cv::Point2f> mFeatures[2];
	double mStartTime;
	cv::Point2f mFaceCenter;
	cv::Point2f mFaceCenterPct;

	ros::Publisher mHeadPosPub;
	ros::Subscriber mHeadSpeedSub;
	ros::Subscriber mHeadPosSub;
	image_transport::Subscriber mImageSub;
	ros::ServiceServer mActiveServer;
	ros::ServiceClient mImageControlClient;
	ros::ServiceClient mQueryCloudClient;
	ros::ServiceClient mForceDepthClient;

	nero_msgs::PitchYaw mCurrentOrientation;
	nero_msgs::PitchYaw mCurrentSpeed;
	bool mActive;
	double mScale;
	bool mDisplayFrames;
	bool mDetectOnly;
	bool mSendHeadPosition;

	inline bool canMoveHead() { return mCurrentSpeed.pitch < STOP_SPEED_TOLERANCE && mCurrentSpeed.yaw < STOP_SPEED_TOLERANCE; };

public:
	FocusFace(const char *frontal_face, const char *profile_face, const char *frontal_face_2);

	void headSpeedCb(const nero_msgs::PitchYaw &msg);
	void headPositionCb(const nero_msgs::PitchYaw &msg);

	void detectFaces(cv::Mat &frame);
	void trackFace(cv::Mat &prevFrame, cv::Mat &frame);
	void imageCb(const sensor_msgs::ImageConstPtr &image);
	bool setActiveCB(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);

	void sendHeadPosition();

	inline ros::NodeHandle *getNodeHandle() { return &mNodeHandle; };
	inline bool isActive() { return mActive; };
	inline void setRGBOutput(bool active) { nero_msgs::SetActive srv; srv.request.active = active; mImageControlClient.call(srv); };
	inline void setDepthForced(bool active) { nero_msgs::SetActive srv; srv.request.active = active; mForceDepthClient.call(srv); };
};

#endif /* FOCUSFACE_H_ */
