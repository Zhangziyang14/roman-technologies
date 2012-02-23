/*
 * FocusFace.h
 *
 *  Created on: Feb 7, 2012
 *      Author: hans
 */

#ifndef FOCUSFACE_H_
#define FOCUSFACE_H_

//#define WEBCAM

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include "pcl/ros/conversions.h"

#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "nero_msgs/PitchYaw.h"
#include "nero_msgs/SetActive.h"

#define STOP_SPEED_TOLERANCE 0.005
#define MAX_CORNERS 200

#define MIN_FEATURE_COUNT 10
#define OUTLIER_THRESHOLD 3.0
#define MAX_TRACKING_TIME 3.0

class FocusFace
{
private:
	ros::NodeHandle mNodeHandle;
	cv::CascadeClassifier mFrontalFaceCascade;
	cv::CascadeClassifier mProfileCascade;
	cv::CascadeClassifier mFrontalFace2Cascade;
	cv::Mat mPrevFrame;
	std::vector<cv::Point2f> mFeatures[2];
	double mStartTime;
	cv::Point2f mFaceCenter;

	ros::Publisher mHeadPosPub;
	ros::Subscriber mHeadSpeedSub;
	ros::Subscriber mHeadPosSub;
	ros::Subscriber mImageSub;
	ros::ServiceServer mActiveServer;

	nero_msgs::PitchYaw mCurrentOrientation;
	nero_msgs::PitchYaw mCurrentSpeed;
	bool mActive;
	double mScale;
	bool mDisplayFrames;

	inline bool canMoveHead() { return mCurrentSpeed.pitch < STOP_SPEED_TOLERANCE && mCurrentSpeed.yaw < STOP_SPEED_TOLERANCE; };

public:
	FocusFace(const char *frontal_face, const char *profile_face, const char *frontal_face_2);

	void headSpeedCb(const nero_msgs::PitchYaw &msg);
	void headPositionCb(const nero_msgs::PitchYaw &msg);

	void detectFaces(cv::Mat &frame, pcl::PointCloud<pcl::PointXYZRGB> cloud);
	void trackFace(cv::Mat &prevFrame, cv::Mat &frame, pcl::PointCloud<pcl::PointXYZRGB> cloud);
#ifdef WEBCAM
	void imageCb(const sensor_msgs::ImagePtr &image_);
#else
	void imageCb(const sensor_msgs::PointCloud2Ptr &cloud2);
#endif
	bool setActiveCB(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);

	void sendHeadPosition(pcl::PointCloud<pcl::PointXYZRGB> cloud);

	inline ros::NodeHandle *getNodeHandle() { return &mNodeHandle; };
	inline bool isActive() { return mActive; };

};

#endif /* FOCUSFACE_H_ */
