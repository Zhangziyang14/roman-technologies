/*
 * KinectServer.h
 *
 *  Created on: 2012-04-18
 *      Author: hgaiser
 */

#ifndef KINECTSERVER_H_
#define KINECTSERVER_H_

#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "image_server/OpenCVTools.h"
#include "sensor_msgs/CompressedImage.h"
#include "nero_msgs/SetActive.h"
#include "nero_msgs/ColorDepth.h"
#include "nero_msgs/QueryCloud.h"
#include "image_server/CaptureKinect.h"

class KinectServer
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	image_transport::ImageTransport mImageTransport;
	image_transport::Publisher mRGBPub;
	ros::Publisher mPCPub;
	ros::Publisher mDepthPub;
	ros::Publisher mLaserPub;
	ros::Publisher mRGBDepthPub;
	ros::Publisher mXYZRGBPub;
	ros::ServiceServer mRGBControl;
	ros::ServiceServer mCloudControl;
	ros::ServiceServer mForceKinectControl;
	ros::ServiceServer mQueryCloud;

	bool mSendEmptyLaserscan;
	bool mPublishRGB;
	bool mPublishDepth;
	bool mPublishCloud;
	bool mPublishLaserScan;
	bool mPublishRGBDepth;
	bool mPublishXYZRGB;

	bool mForceKinectOpen;

	double mScale;
	bool mCloseIdleKinect;

	CaptureKinect mKinect;

public:
	KinectServer(const char *filePath);
	~KinectServer()
	{
		mNodeHandle.shutdown();
	}

	void run();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
	inline bool isDepthGenerating() { return mKinect.isDepthGenerating(); };
	inline bool isRGBGenerating() { return mKinect.isRGBGenerating(); };
	inline bool isGenerating() { return isDepthGenerating() || isRGBGenerating(); };

private:
	bool RGBControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool CloudControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool ForceKinectControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);

	bool QueryCloud(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res);
};

#endif /* KINECTSERVER_H_ */
