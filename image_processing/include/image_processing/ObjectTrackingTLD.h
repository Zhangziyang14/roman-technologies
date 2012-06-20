/*
 * ObjectTrackingTLD.h
 *
 *  Created on: May 22, 2012
 *      Author: hans
 */

#ifndef OBJECTTRACKINGTLD_H_
#define OBJECTTRACKINGTLD_H_

#include "ros/ros.h"

#include "tld/TLD.h"
#include "image_transport/image_transport.h"

#include "image_server/OpenCVTools.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"

#include "nero_msgs/SetActive.h"
#include "nero_msgs/ObjectModel.h"

class ObjectTrackingTLD
{
private:
	ros::NodeHandle mNodeHandle;
	image_transport::ImageTransport mImageTransport;

	image_transport::Subscriber mImageSub;
	ros::Publisher mObjPosPub;

	ros::ServiceServer mLoadModelServer;
	ros::ServiceServer mSetTrackingServer;
	ros::ServiceServer mSetLearningServer;
	ros::ServiceServer mSetDetectingServer;

	tld::TLD *mTLD;

	bool mWaitForBBox;
	cv::Mat mLastImage;
	std::string mModelPath;

	cv::Scalar mYellow;
	cv::Scalar mBlue;

	nero_msgs::ObjectModel::Request mObjectModel;

	void sendPosition(cv::Rect r);

public:
	ObjectTrackingTLD();

	void imageCb(const sensor_msgs::ImageConstPtr &image);

	bool loadModelCb(nero_msgs::ObjectModel::Request &req, nero_msgs::ObjectModel::Response &res);
	bool setTrackingCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool setLearningCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool setDetectingCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);

	void spin();
};

#endif /* OBJECTTRACKINGTLD_H_ */
