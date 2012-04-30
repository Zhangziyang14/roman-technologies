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

class KinectServer
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	//image_transport::ImageTransport mImageTransport;
	//image_transport::Publisher mRGBPub;
	ros::Publisher mRGBPub;
	ros::Publisher mPCPub;
	ros::Publisher mLaserPub;
	ros::ServiceServer mRGBControl;
	ros::ServiceServer mCloudControl;

	bool mSendEmptyLaserscan;
	bool mPublishRGB;
	bool mPublishCloud;
	bool mPublishLaserScan;

	double mScale;
	bool mCloseIdleKinect;

	cv::VideoCapture mKinect;

public:
	KinectServer();
	~KinectServer()
	{
		mNodeHandle.shutdown();
	}

	void run();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
	inline bool isCapturing() { return mKinect.isOpened(); };

private:
	bool RGBControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool CloudControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
};

#endif /* KINECTSERVER_H_ */
