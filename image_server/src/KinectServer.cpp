#include "image_server/KinectServer.h"

KinectServer::KinectServer(const char *filePath) :
	mNodeHandle(""),
	mImageTransport(mNodeHandle),
	mPublishLaserScan(false),
	mForceKinectOpen(false),
	mForceDepthOpen(false),
	mKinect(filePath)
{
	mRGBPub				= mImageTransport.advertise("/camera/rgb/image_color", 1);
	mPCPub				= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
	mDepthPub			= mNodeHandle.advertise<sensor_msgs::Image>("/camera/depth/image", 1);
	mLaserPub			= mNodeHandle.advertise<sensor_msgs::LaserScan>("/camera/scan", 1);
	mRGBDepthPub		= mNodeHandle.advertise<nero_msgs::ColorDepth>("/camera/color_depth", 1);
	mXYZRGBPub			= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/rgb/points", 1);
	mRGBControl			= mNodeHandle.advertiseService("/KinectServer/RGBControl", &KinectServer::RGBControl, this);
	mCloudControl		= mNodeHandle.advertiseService("/KinectServer/CloudControl", &KinectServer::CloudControl, this);
	mForceKinectControl	= mNodeHandle.advertiseService("/KinectServer/ForceKinectControl", &KinectServer::ForceKinectControl, this);
	mForceDepthControl	= mNodeHandle.advertiseService("/KinectServer/ForceDepthControl", &KinectServer::ForceDepthControl, this);
	mQueryCloud			= mNodeHandle.advertiseService("/KinectServer/QueryCloud", &KinectServer::QueryCloud, this);
	mProjectPoints		= mNodeHandle.advertiseService("/KinectServer/ProjectPoints", &KinectServer::ProjectPoints, this);
	
	mNodeHandle.param<bool>("/KinectServer/send_empty_laserscan", mSendEmptyLaserscan, false);
	mNodeHandle.param<double>("/KinectServer/scale", mScale, 1.0);
	mNodeHandle.param<bool>("/KinectServer/publish_rgb", mPublishRGB, true);
	mNodeHandle.param<bool>("/KinectServer/publish_depth", mPublishDepth, true);
	mNodeHandle.param<bool>("/KinectServer/publish_cloud", mPublishCloud, true);
	mNodeHandle.param<bool>("/KinectServer/publish_rgb_depth", mPublishRGBDepth, true);
	mNodeHandle.param<bool>("/KinectServer/publish_xyzrgb", mPublishXYZRGB, true);
	mNodeHandle.param<bool>("/KinectServer/close_idle_kinect", mCloseIdleKinect, true);
	
	ROS_INFO("KinectServer initialised.");
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void KinectServer::run()
{
	bool publishRealLaserScan 	= mSendEmptyLaserscan == false && mPublishLaserScan && mLaserPub.getNumSubscribers();
	bool publishRGB 			= mPublishRGB && mRGBPub.getNumSubscribers();
	bool publishDepth 			= mPublishDepth && mDepthPub.getNumSubscribers();
	bool publishRGBDepth		= mPublishRGBDepth && mRGBDepthPub.getNumSubscribers();
	bool publishCloud			= mPublishCloud && mPCPub.getNumSubscribers();
	bool publishXYZRGB			= mPublishXYZRGB && mXYZRGBPub.getNumSubscribers();

	bool captureRGB = publishRGB || publishRGBDepth || publishXYZRGB;
	bool captureDepth = publishCloud || publishRealLaserScan || publishRGBDepth || publishDepth || publishXYZRGB || mForceDepthOpen;

	if (mKinect.isOpened() == false)
	{
		ROS_INFO("Opening Kinect...");

		mKinect.open();
		if (mKinect.isOpened() == false)
		{
			ROS_WARN("Can not open Kinect device, will try again in one second.");
			usleep(1000000);
			return;
		}

		ROS_INFO("Kinect opened.");
	}

	if (mForceKinectOpen == false && mCloseIdleKinect && isGenerating())
	{
		if (captureRGB == false && isRGBGenerating())
		{
			mKinect.stopRGB(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Stopped RGB stream.");
		}

		if (captureDepth == false && isDepthGenerating())
		{
			mKinect.stopDepth(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Stopped depth stream.");
		}

		// nothing to do
		if (isGenerating() == false)
			return;
	}

	// start streaming RGB if required
	if ((mForceKinectOpen || captureRGB) && isRGBGenerating() == false)
	{
		ROS_INFO("Starting RGB stream ...");
		mKinect.startRGB();
		ROS_INFO("Started RGB stream.");
	}

	// start streaming depth if required
	if ((mForceKinectOpen || captureDepth) && isDepthGenerating() == false)
	{
		ROS_INFO("Starting depth stream ...");
		mKinect.startDepth();
		ROS_INFO("Started depth stream.");
	}

	// we're not actually capturing anything now
	if (captureRGB == false && captureDepth == false)
		return;

	if (mKinect.queryFrame(captureRGB, captureDepth) == false)
	{
		ROS_WARN("Can not grab images, releasing Kinect.");
		mKinect.close();
		return;
	}

	cv::Mat rgb;
	if(captureRGB)
	{
		rgb = mKinect.getImage();
		if (rgb.empty())
		{
			ROS_WARN("Failed to grab RGB image.");
			return;
		}
	}

	if (publishRGB)
	{
		if (mScale != 1.0)
			cv::resize(rgb, rgb, cv::Size(rgb.cols * mScale, rgb.rows * mScale), 0, 0, cv::INTER_LINEAR);
		
		if (publishRGB)
			mRGBPub.publish(OpenCVTools::matToImage(rgb));
	}

	cv::Mat depth;
	if(captureDepth)
	{
		depth = mKinect.getDepth();
		if (depth.empty())
		{
			ROS_WARN("Failed to grab Depth image.");
			return;
		}
	}

	cv::Mat cloud;
	if (publishCloud || publishXYZRGB)
	{
		cloud = mKinect.getCloud(depth);
		if (cloud.empty())
		{
			ROS_WARN("Failed to grab RGB image.");
			return;
		}

		if (publishCloud)
			mPCPub.publish(OpenCVTools::matToPointCloud2(cloud));
		if (publishXYZRGB)
			mXYZRGBPub.publish(OpenCVTools::matToRegisteredPointCloud2(cloud, rgb));
	}

	if (mPublishLaserScan)
		mLaserPub.publish(OpenCVTools::matToLaserScan(cloud, mSendEmptyLaserscan));

	if (publishDepth) {
		mDepthPub.publish(OpenCVTools::matToImage(depth));
	}

	if (publishRGBDepth)
	{
		nero_msgs::ColorDepth msg;
		msg.color = *OpenCVTools::matToImage(rgb);
		msg.depth = *OpenCVTools::matToImage(depth);
		mRGBDepthPub.publish(msg);
	}
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

bool KinectServer::ForceDepthControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mForceDepthOpen == false && req.active)
		ROS_INFO("Forcing depth open");
	else if (mForceDepthOpen && req.active == false)
		ROS_INFO("No longer forcing depth");

	mForceDepthOpen = req.active;
	return true;
}

bool KinectServer::QueryCloud(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res)
{
	if (mKinect.isOpened() == false)
	{
		ROS_WARN("Received cloud query while Kinect is offline");
		return false;
	}

	if (isDepthGenerating() == false)
		mKinect.startDepth();

	mKinect.queryFrame(false, true);

	cv::Mat cloud = mKinect.getCloud(mKinect.getDepth());
	if (cloud.cols == 0 || cloud.rows == 0)
	{
		ROS_WARN("Failed to retrieve cloud image during cloud query.");
		return false;
	}

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/kinect_frame";
	geometry_msgs::PointStamped invalid;
	invalid.point.x = invalid.point.y = invalid.point.z = std::numeric_limits<float>::quiet_NaN();
	for (size_t i = 0; i < req.points.size(); i++)
	{
		if (req.points[i].x >= 0.0 && req.points[i].x < 1.0 &&
			req.points[i].y >= 0.0 && req.points[i].y < 1.0)
		{
			geometry_msgs::PointStamped gp;
			gp.header = header;
			cv::Point3f cp = cloud.at<cv::Point3f>(req.points[i].y * cloud.rows, req.points[i].x * cloud.cols);
			gp.point.x = cp.x; gp.point.y = cp.y; gp.point.z = cp.z;
			res.points.push_back(gp);
		}
		else
		{
			res.points.push_back(invalid);
		}
	}

	return true;
}

bool KinectServer::ProjectPoints(nero_msgs::QueryCloud::Request &req, nero_msgs::QueryCloud::Response &res)
{
	if (mKinect.isOpened() == false)
	{
		ROS_WARN("Received cloud query while Kinect is offline");
		return false;
	}

    cv::Ptr<XnPoint3D> proj = new XnPoint3D[req.points.size()];
    cv::Ptr<XnPoint3D> real = new XnPoint3D[req.points.size()];

    for (size_t i = 0; i < req.points.size(); i++)
    {
        proj[i].X = req.points[i].x;
        proj[i].Y = req.points[i].y;
        proj[i].Z = req.points[i].z;
    }

    mKinect.getDepthGenerator()->ConvertProjectiveToRealWorld(req.points.size(), proj, real);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/kinect_frame";
	geometry_msgs::PointStamped invalid;
	invalid.point.x = invalid.point.y = invalid.point.z = std::numeric_limits<float>::quiet_NaN();
	invalid.header = header;
	for (size_t i = 0; i < req.points.size(); i++)
	{
		if (req.points[i].z)
		{
			geometry_msgs::PointStamped p;
			p.header = header;
			p.point.x = real[i].X * 0.001f;
			p.point.y = real[i].Y * 0.001f;
			p.point.z = real[i].Z * 0.001f;
			res.points.push_back(p);
		}
		else
			res.points.push_back(invalid);
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
		
		if (show_fps && kinectServer.isGenerating())
		{
			double cycleTime = rate.cycleTime().toSec();
			if (cycleTime != 0.0)
			{
				fps += (1.0 / cycleTime);
				fpsCount++;
				
				if (fpsCount == 10)
				{
//					ROS_INFO("Rate: %lf", std::min(desired_fps, fps / fpsCount));
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

