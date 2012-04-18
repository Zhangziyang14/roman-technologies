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

class KinectServer
{
protected:
	ros::NodeHandle mNodeHandle;				//ROS node handler
	image_transport::ImageTransport mImageTransport;
	image_transport::Publisher mRGBPub;
	ros::Publisher mPCPub;
	ros::Publisher mLaserPub;
	bool mSendEmptyLaserscan;
	bool mPublishRGB;
	bool mPublishCloud;
	bool mPublishLaserScan;

	cv::VideoCapture mKinect;

public:
	KinectServer();
	~KinectServer()
	{
		mNodeHandle.shutdown();
	}

	void run();

	inline ros::NodeHandle* getNodeHandle() { return &mNodeHandle; };
};

#endif /* KINECTSERVER_H_ */
