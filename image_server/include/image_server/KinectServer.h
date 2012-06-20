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
#include "nero_msgs/GetCloud.h"
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
	ros::Publisher mFilteredRGBPub;
	ros::Publisher mFilteredRGBDepthPub;
	ros::Publisher mXYZRGBPub;
	ros::ServiceServer mRGBControl;
	ros::ServiceServer mCloudControl;
	ros::ServiceServer mForceKinectControl;
	ros::ServiceServer mCloudServer;
	ros::ServiceServer mQueryCloud;
	ros::ServiceServer mProjectPoints;

	bool mPublishRGB;
	bool mPublishDepth;
	bool mPublishCloud;
	bool mPublishLaserScan;
	bool mPublishRGBDepth;
	bool mPublishFilteredRGB;
	bool mPublishFilteredRGBDepth;
	bool mPublishXYZRGB;

	bool mForceKinectOpen;
	bool mForceDepth;

	int mFilterDistance;
	double mScale;
	bool mCloseIdleKinect;

	//cv::VideoCapture mKinect;
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
	bool openKinect();
	bool queryKinect(bool queryRGB, bool queryDepth);
	bool grabRGB(cv::Mat &rgb);
	bool grabDepth(cv::Mat &depth);
	bool grabCloud(cv::Mat &cloud, cv::Mat depth);
	void filterRGBDepth(cv::Mat rgb_in, cv::Mat depth_in, cv::Mat &rgb_out, cv::Mat &depth_out, bool filterDepth = false);

	inline void startRGB()
	{
		if (isRGBGenerating())
			return;

		ROS_INFO("Starting RGB stream ...");
		mKinect.startRGB();
		ROS_INFO("Started RGB stream.");
	};

	inline void startDepth()
	{
		if (isDepthGenerating())
			return;

		ROS_INFO("Starting depth stream ...");
		mKinect.startDepth();
		ROS_INFO("Started depth stream.");
	};

	inline void resizeMat(cv::Mat &mat, float scale = 1.f) { if (scale == 1.f) return; cv::resize(mat, mat, cv::Size(mat.cols * mScale, mat.rows * scale), 0, 0, cv::INTER_LINEAR); };
	inline void publishRGB(cv::Mat rgb) { resizeMat(rgb, mScale); mRGBPub.publish(OpenCVTools::matToImage(rgb)); };
	inline void publishDepth(cv::Mat depth) { resizeMat(depth, mScale); mDepthPub.publish(OpenCVTools::matToImage(depth)); };
	inline void publishCloud(cv::Mat cloud) { mPCPub.publish(OpenCVTools::matToPointCloud2(cloud)); };
	inline void publishRegisteredCloud(cv::Mat cloud, cv::Mat rgb) { mXYZRGBPub.publish(OpenCVTools::matToRegisteredPointCloud2(cloud, rgb)); };
	inline void publishLaserScan(cv::Mat cloud) { mLaserPub.publish(OpenCVTools::matToLaserScan(cloud)); };
	inline void publishRGBDepth(cv::Mat rgb, cv::Mat depth)
	{
		nero_msgs::ColorDepth msg;
		msg.color = *OpenCVTools::matToImage(rgb);
		msg.depth = *OpenCVTools::matToImage(depth);
		mRGBDepthPub.publish(msg);
	};
	inline void publishFiltered(cv::Mat rgb, cv::Mat depth, bool publishRGB, bool publishRGBDepth)
	{
		cv::Mat filtered_rgb;
		cv::Mat filtered_depth;

		filterRGBDepth(rgb, depth, filtered_rgb, filtered_depth, publishRGBDepth);

		if(publishRGBDepth)
		{
			nero_msgs::ColorDepth msg;
			msg.color = *OpenCVTools::matToImage(filtered_rgb);
			msg.depth = *OpenCVTools::matToImage(filtered_depth);
			mFilteredRGBDepthPub.publish(msg);
		}

		if(publishRGB)
			mFilteredRGBPub.publish(OpenCVTools::matToImage(filtered_rgb));
	};

	bool RGBControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool CloudControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool ForceKinectControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);
	bool CloudServer(nero_msgs::GetCloud::Request &req, nero_msgs::GetCloud::Response &res);
	bool ForceDepth(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res);

	bool QueryCloud(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res);
	bool ProjectPoints(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res);
};

#endif /* KINECTSERVER_H_ */

