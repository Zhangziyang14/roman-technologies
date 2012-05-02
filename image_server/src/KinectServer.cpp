#include "image_server/KinectServer.h"

KinectServer::KinectServer() :
	mNodeHandle(""),
	mPublishLaserScan(false),
	mForceKinectOpen(false)
	//mImageTransport(mNodeHandle)
{
	mNodeHandle.param<bool>("/KinectServer/send_empty_laserscan", mSendEmptyLaserscan, false);
	
	mRGBPub				= mNodeHandle.advertise<sensor_msgs::CompressedImage>("/camera/rgb/image_color/compressed", 1);
	mPCPub				= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
	mLaserPub			= mNodeHandle.advertise<sensor_msgs::LaserScan>("/scan", 1);
	mRGBControl			= mNodeHandle.advertiseService("/KinectServer/RGBControl", &KinectServer::RGBControl, this);
	mCloudControl		= mNodeHandle.advertiseService("/KinectServer/CloudControl", &KinectServer::CloudControl, this);
	mForceKinectControl	= mNodeHandle.advertiseService("/KinectServer/ForceKinectControl", &KinectServer::ForceKinectControl, this);
	mQueryCloud			= mNodeHandle.advertiseService("/KinectServer/QueryCloud", &KinectServer::QueryCloud, this);
	
	mNodeHandle.param<double>("/KinectServer/scale", mScale, 1.0);
	mNodeHandle.param<bool>("/KinectServer/publish_rgb", mPublishRGB, false);
	mNodeHandle.param<bool>("/KinectServer/publish_cloud", mPublishCloud, false);
	mNodeHandle.param<bool>("/KinectServer/close_idle_kinect", mCloseIdleKinect, true);
	
	ROS_INFO("done.");
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void KinectServer::run()
{
	bool publishRealLaserScan = mSendEmptyLaserscan == false && mPublishLaserScan;
	bool captureRGB = mPublishRGB && mRGBPub.getNumSubscribers();
	bool captureCloud = (mPublishCloud && mPCPub.getNumSubscribers()) || (publishRealLaserScan && mLaserPub.getNumSubscribers());
	
	// nothing to do right now
	if (captureRGB == false && captureCloud == false && mForceKinectOpen == false)
	{
		if (mCloseIdleKinect && mKinect.isOpened())
		{
			mKinect.release(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Kinect closed.");
		}
		return;
	}

	if (mKinect.isOpened() == false)
	{
		ROS_INFO("Opening Kinect.");
		mKinect.open(CV_CAP_OPENNI);
		if (mKinect.isOpened() == false)
		{
			ROS_WARN("Can not open Kinect device, will try again in one second.");
			usleep(1000000);
			return;
		}

		mKinect.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default
		ROS_INFO("Kinect opened.");
	}

	// we're not actually capturing anything now
	if (captureRGB == false && captureCloud == false)
		return;

	if (mKinect.grab() == false)
	{
		ROS_WARN("Can not grab images.");
		return;
	}

	cv::Mat rgb;
	if (captureRGB)
	{
		if (mKinect.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE) == false)
		{
			ROS_WARN("Failed to retrieve RGB image.");
			return;
		}

		if (mScale != 1.0)
			cv::resize(rgb, rgb, cv::Size(rgb.cols * mScale, rgb.rows * mScale), 0, 0, cv::INTER_LINEAR);
		
		//if (mPublishRGB)
		//	mRGBPub.publish(OpenCVTools::matToImage(rgb));
		
		sensor_msgs::CompressedImage rgb_c;
		rgb_c.header.stamp = ros::Time::now();
		std::vector<int> p;
		p.push_back(CV_IMWRITE_JPEG_QUALITY);
		p.push_back(90);
		cv::imencode(".jpg", rgb, rgb_c.data, p);
		
		mRGBPub.publish(rgb_c);
	}

	cv::Mat cloud;
	if (captureCloud)
	{
		if (mKinect.retrieve(cloud, CV_CAP_OPENNI_POINT_CLOUD_MAP) == false)
		{
			ROS_WARN("Failed to retrieve cloud image.");
			return;
		}

		if (mPublishCloud)
			mPCPub.publish(OpenCVTools::matToPointCloud2(cloud));
	}

	if (mPublishLaserScan)
		mLaserPub.publish(OpenCVTools::matToLaserScan(cloud, mSendEmptyLaserscan));
}

bool KinectServer::RGBControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mPublishRGB == false && req.active)
		ROS_INFO("Publishing RGB images");
	else if (mPublishRGB && req.active == false)
		ROS_INFO("No longer publishing RGB images");

	mPublishRGB = req.active;
	return true;
}

bool KinectServer::CloudControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mPublishCloud == false && req.active)
		ROS_INFO("Publishing clouds");
	else if (mPublishCloud && req.active == false)
		ROS_INFO("No longer publishing clouds");

	mPublishCloud = req.active;
	return true;
}

bool KinectServer::ForceKinectControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mForceKinectOpen == false && req.active)
		ROS_INFO("Saving clouds");
	else if (mForceKinectOpen && req.active == false)
		ROS_INFO("No longer saving clouds");

	mForceKinectOpen = req.active;
	return true;
}

bool KinectServer::QueryCloud(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res)
{
	if (mKinect.isOpened() == false)
	{
		ROS_WARN("Received cloud query while Kinect is offline");
		return false;
	}

	cv::Mat cloud;
	if (mKinect.retrieve(cloud, CV_CAP_OPENNI_POINT_CLOUD_MAP) == false)
	{
		ROS_WARN("Failed to retrieve cloud image during cloud query.");
		return false;
	}

	for (size_t i = 0; i < req.indices.size(); i++)
	{
		if (req.indices[i].x >= 0 && req.indices[i].x < cloud.cols &&
			req.indices[i].y >= 0 && req.indices[i].y < cloud.rows)
		{
			geometry_msgs::Point gp;
			cv::Point3f cp = cloud.at<cv::Point3f>(req.indices[i].x, req.indices[i].y);
			gp.x = cp.x; gp.y = cp.y; gp.z = cp.z;
			res.points.push_back(gp);
		}
	}
	return true;
}

int main(int argc, char* argv[])
{
	// init ros
	ros::init(argc, argv, "KinectServer");
	
	KinectServer kinectServer;

	ros::Rate rate(30);

	double fps = 0.0;
	int fpsCount = 0;

	while (ros::ok())
	{
		rate.sleep();
		
		if (kinectServer.isCapturing())
		{
			double cycleTime = rate.cycleTime().toSec();
			if (cycleTime != 0.0)
			{
				fps += (1.0 / cycleTime);
				fpsCount++;
				
				if (fpsCount == 10)
				{
					ROS_INFO("Rate: %lf", fps / fpsCount);
					fps = 0.0;
					fpsCount = 0;
				}
			}
		}
		
		kinectServer.run();
		ros::spinOnce();
	}
	return 0;
}

