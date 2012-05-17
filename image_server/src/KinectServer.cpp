#include "image_server/KinectServer.h"

KinectServer::KinectServer(const char *filePath) :
	mNodeHandle(""),
	mPublishLaserScan(false),
	mForceKinectOpen(false),
	mKinect(filePath)
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
	bool captureCloud = (mPublishCloud) || (publishRealLaserScan && mLaserPub.getNumSubscribers());
	
	if (mKinect.isOpened() == false)
	{
		mKinect.open();
		if (mKinect.isOpened() == false)
		{
			ROS_WARN("Can not open Kinect device, will try again in one second.");
			usleep(1000000);
			return;
		}

		ROS_INFO("Kinect opened.");
	}

	// nothing to do right now
	if (captureRGB == false && captureCloud == false && mForceKinectOpen == false)
	{
		if (mCloseIdleKinect && mKinect.isGenerating())
		{
			mKinect.stop(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Stopped Kinect.");
		}
		return;
	}

	if (mKinect.isGenerating() == false)
	{
		mKinect.start();
		ROS_INFO("Started Kinect.");
	}

	// we're not actually capturing anything now
	if (captureRGB == false && captureCloud == false)
		return;

	//if (mKinect.grab() == false)
	if (mKinect.queryFrame(captureRGB, captureCloud) == false)
	{
		ROS_WARN("Can not grab images, releasing Kinect.");
		mKinect.close();
		return;
	}

	if (captureRGB)
	{
		cv::Mat rgb = mKinect.getImage();
		if (rgb.cols == 0 || rgb.rows == 0)
		{
			ROS_WARN("Failed to grab RGB image.");
			return;
		}

		if (mScale != 1.0)
			cv::resize(rgb, rgb, cv::Size(rgb.cols * mScale, rgb.rows * mScale), 0, 0, cv::INTER_LINEAR);
		
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
		cloud = mKinect.getCloud();
		if (cloud.cols == 0 || cloud.rows == 0)
		{
			ROS_WARN("Failed to grab RGB image.");
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
		ROS_INFO("Forcing Kinect open");
	else if (mForceKinectOpen && req.active == false)
		ROS_INFO("No longer forcing Kinect");

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

	if (mKinect.isGenerating() == false)
		mKinect.start();

	mKinect.queryFrame(false, true);

	cv::Mat cloud = mKinect.getCloud();
	if (cloud.cols == 0 || cloud.rows == 0)
	{
		ROS_WARN("Failed to retrieve cloud image during cloud query.");
		return false;
	}

	geometry_msgs::Point invalid;
	invalid.x = invalid.y = invalid.z = std::numeric_limits<float>::quiet_NaN();
	for (size_t i = 0; i < req.indices.size(); i++)
	{
		if (req.indices[i].x >= 0.0 && req.indices[i].x < 1.0 &&
			req.indices[i].y >= 0.0 && req.indices[i].y < 1.0)
		{
			geometry_msgs::Point gp;
			cv::Point3f cp = cloud.at<cv::Point3f>(req.indices[i].y * cloud.rows, req.indices[i].x * cloud.cols);
			gp.x = cp.x; gp.y = cp.y; gp.z = cp.z;
			res.points.push_back(gp);
		}
		else
		{
			res.points.push_back(invalid);
		}
	}

	return true;
}

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cout << "Usage: ./KinectServer <path-to-xml>" << std::endl;
		return 0;
	}

	// init ros
	ros::init(argc, argv, "KinectServer");
	
	KinectServer kinectServer(argv[1]);

	bool show_fps;
	double desired_fps;
	kinectServer.getNodeHandle()->param<bool>("/KinectServer/show_fps", show_fps, false);
	kinectServer.getNodeHandle()->param<double>("/KinectServer/fps", desired_fps, 30.0);
	double fps = 0.0;
	int fpsCount = 0;

	ros::Rate rate(desired_fps);

	while (ros::ok())
	{
		rate.sleep();
		
		if (show_fps && kinectServer.isCapturing())
		{
			double cycleTime = rate.cycleTime().toSec();
			if (cycleTime != 0.0)
			{
				fps += (1.0 / cycleTime);
				fpsCount++;
				
				if (fpsCount == 10)
				{
					ROS_INFO("Rate: %lf", std::min(desired_fps, fps / fpsCount));
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

