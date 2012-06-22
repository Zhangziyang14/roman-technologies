#include "image_server/KinectServer.h"

/**
 * Constructor
 */
KinectServer::KinectServer(const char *filePath) :
	mNodeHandle(""),
	mImageTransport(mNodeHandle),
	mPublishLaserScan(false),
	mForceKinectOpen(false),
	mForceDepth(false),
	mKinect(filePath)
{
	mRGBPub				= mImageTransport.advertise("/camera/rgb/image_color", 1);
	mPCPub				= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
	mDepthPub			= mNodeHandle.advertise<sensor_msgs::Image>("/camera/depth/image", 1);
	mLaserPub			= mNodeHandle.advertise<sensor_msgs::LaserScan>("/camera/scan", 1);
	mRGBDepthPub		= mNodeHandle.advertise<nero_msgs::ColorDepth>("/camera/color_depth", 1);
	mFilteredRGBPub		= mNodeHandle.advertise<sensor_msgs::Image>("/camera/filtered/color", 1);
	mFilteredRGBDepthPub= mNodeHandle.advertise<nero_msgs::ColorDepth>("/camera/filtered/color_depth", 1);
	mXYZRGBPub			= mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/rgb/points", 1);
	mRGBControl			= mNodeHandle.advertiseService("/KinectServer/RGBControl", &KinectServer::RGBControl, this);
	mCloudControl		= mNodeHandle.advertiseService("/KinectServer/CloudControl", &KinectServer::CloudControl, this);
	mForceKinectControl	= mNodeHandle.advertiseService("/KinectServer/ForceKinectControl", &KinectServer::ForceKinectControl, this);
	mCloudServer		= mNodeHandle.advertiseService("/KinectServer/CloudServer", &KinectServer::CloudServer, this);
	mForceDepthControl	= mNodeHandle.advertiseService("/KinectServer/ForceDepth", &KinectServer::ForceDepth, this);
	mQueryCloud			= mNodeHandle.advertiseService("/KinectServer/QueryCloud", &KinectServer::QueryCloud, this);
	mProjectPoints		= mNodeHandle.advertiseService("/KinectServer/ProjectPoints", &KinectServer::ProjectPoints, this);

	mNodeHandle.param<int>("/KinectServer/filter_distance", mFilterDistance, 2000);
	mNodeHandle.param<double>("/KinectServer/scale", mScale, 1.0);
	mNodeHandle.param<bool>("/KinectServer/publish_rgb", mPublishRGB, true);
	mNodeHandle.param<bool>("/KinectServer/publish_depth", mPublishDepth, true);
	mNodeHandle.param<bool>("/KinectServer/publish_cloud", mPublishCloud, true);
	mNodeHandle.param<bool>("/KinectServer/publish_rgb_depth", mPublishRGBDepth, true);
	mNodeHandle.param<bool>("/KinectServer/publish_filtered_rgb", mPublishFilteredRGB, true);
	mNodeHandle.param<bool>("/KinectServer/publish_xyzrgb", mPublishXYZRGB, true);
	mNodeHandle.param<bool>("/KinectServer/close_idle_kinect", mCloseIdleKinect, true);

	ROS_INFO("KinectServer initialised.");
}

/**
 * Attempts to open the Kinect device. Waits one second and returns false if it fails to open.
 */
bool KinectServer::openKinect()
{
	if (mKinect.isOpened() == false)
	{
		ROS_INFO("Opening Kinect ...");

		mKinect.open();
		if (mKinect.isOpened() == false)
		{
			ROS_WARN("Can not open Kinect device, will try again in one second.");
			usleep(1000000);
			return false;
		}

		ROS_INFO("Kinect opened.");
	}

	return true;
}

/**
 * Query the kinect for new images. Will start streaming if not streaming already.
 */
bool KinectServer::queryKinect(bool queryRGB, bool queryDepth)
{
	// is the kinect open or can we open it?
	if (openKinect() == false)
		return false;

	// stop generating if no longer necessary
	if (mForceKinectOpen == false && mCloseIdleKinect && isGenerating())
	{
		if (queryRGB == false && isRGBGenerating())
		{
			mKinect.stopRGB(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Stopped RGB stream.");
		}

		if (queryDepth == false && isDepthGenerating())
		{
			mKinect.stopDepth(); // lets not waste CPU usage on the Kinect
			ROS_INFO("Stopped depth stream.");
		}

		// nothing to do
		if (isGenerating() == false)
			return false;
	}

	// start streaming RGB if required
	if (mForceKinectOpen || queryRGB)
		startRGB();

	// start streaming depth if required
	if (mForceKinectOpen || queryDepth)
		startDepth();

	// we're not actually capturing anything right now
	if (queryRGB == false && queryDepth == false)
		return false;

	if (mKinect.queryFrame(queryRGB, queryDepth) == false)
	{
		ROS_WARN("Can not grab images, releasing Kinect.");
		mKinect.close();
		return false;
	}

	return true;
}

/**
 * Fills rgb with the RGB image from the Kinect.
 */
bool KinectServer::grabRGB(cv::Mat &rgb)
{
	rgb = mKinect.getImage();
	if (rgb.empty())
	{
		ROS_WARN("Failed to grab RGB image.");
		return false;
	}

	return true;
}

/**
 * Fills depth with the depth image from the Kinect.
 */
bool KinectServer::grabDepth(cv::Mat &depth)
{
	depth = mKinect.getDepth();
	if (depth.empty())
	{
		ROS_WARN("Failed to grab depth image.");
		return false;
	}

	return true;
}

/**
 * Projects the depth image to real world points, using the Kinect device.
 */
bool KinectServer::grabCloud(cv::Mat &cloud, cv::Mat depth)
{
	cloud = mKinect.getCloud(depth);
	if (cloud.empty())
	{
		ROS_WARN("Failed to grab point cloud.");
		return false;
	}

	return true;
}

/**
 * Filters the RGB and or Depth image by filterDepth (default is set to mFilterDepth)
 */
void KinectServer::filterRGBDepth(cv::Mat rgb_in, cv::Mat depth_in, cv::Mat &rgb_out, cv::Mat &depth_out, bool filterDepth)
{
	// create a structuring element for dilation
    cv::Mat se = cv::getStructuringElement(0, cv::Size(15,15));

    // make a mask based on the depth
	cv::Mat mask = (depth_in <= mFilterDistance) & (depth_in != 0);

	// dilate the mask so all important parts are fully in the mask
	cv::dilate(mask, mask, se);
	rgb_in.copyTo(rgb_out, mask);

	if (filterDepth)
		depth_in.copyTo(depth_out, mask);
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void KinectServer::run()
{
	bool sendLaserScan 			= mPublishLaserScan && mLaserPub.getNumSubscribers();
	bool sendRGB 				= mPublishRGB && mRGBPub.getNumSubscribers();
	bool sendDepth 				= mPublishDepth && mDepthPub.getNumSubscribers();
	bool sendRGBDepth			= mPublishRGBDepth && mRGBDepthPub.getNumSubscribers();
	bool sendFilteredRGB		= mPublishFilteredRGB && mFilteredRGBPub.getNumSubscribers();
	bool sendFilteredRGBDepth	= mPublishFilteredRGBDepth && mFilteredRGBDepthPub.getNumSubscribers();
	bool sendCloud				= mPublishCloud && mPCPub.getNumSubscribers();
	bool sendXYZRGB				= mPublishXYZRGB && mXYZRGBPub.getNumSubscribers();

	// determine what we need from the kinect
	bool captureRGB = sendRGB || sendRGBDepth || sendFilteredRGB || sendXYZRGB || sendFilteredRGBDepth;
	bool captureCloud = sendCloud || sendXYZRGB || sendLaserScan;
	bool captureDepth = captureCloud || sendRGBDepth || sendFilteredRGB || sendDepth || sendFilteredRGBDepth || mForceDepth;

	// retrieve the images from the kinect
	if (queryKinect(captureRGB, captureDepth) == false)
		return;

	// attempt to grab the RGB image
	cv::Mat rgb;
	if(captureRGB && grabRGB(rgb) == false)
		return;

	// attempt to grab the depth image
	cv::Mat depth;
	if(captureDepth && grabDepth(depth) == false)
		return;

	// publish the image
	if (sendRGB)
		publishRGB(rgb);

	// publish the depth
	if (sendDepth)
		publishDepth(depth);

	// combine and publish the rgb and depth in one message
	if (sendRGBDepth)
		publishRGBDepth(rgb, depth);

	// grab a cloud
	cv::Mat cloud;
	if (captureCloud && grabCloud(cloud, depth) == false)
		return;

	// publish the cloud
	if (sendCloud)
		publishCloud(cloud);

	// publish the filtered cloud (each point has XYZ and RGB)
	if (sendXYZRGB)
		publishRegisteredCloud(cloud, rgb);

	// publish the laserscan (used for navigating)
	if (sendLaserScan)
		publishLaserScan(cloud);

	// publish the filtered RGB and or RGBDepth
	if (sendFilteredRGB || sendFilteredRGBDepth)
		publishFiltered(rgb, depth, sendFilteredRGB, sendFilteredRGBDepth);
}

/**
 * Service handler for enabling/disabling RGB output.
 */
bool KinectServer::RGBControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mPublishRGB == false && req.active)
		ROS_INFO("Publishing RGB images");
	else if (mPublishRGB && req.active == false)
		ROS_INFO("No longer publishing RGB images");

	mPublishRGB = req.active;
	return true;
}

/**
 * Service handler for enabling/disabling Cloud output.
 */
bool KinectServer::CloudControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mPublishCloud == false && req.active)
		ROS_INFO("Publishing clouds");
	else if (mPublishCloud && req.active == false)
		ROS_INFO("No longer publishing clouds");

	mPublishCloud = req.active;
	return true;
}

/**
 * Service handler for enabling/disabling the forcing of keeping the Kinect opened.
 */
bool KinectServer::ForceKinectControl(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mForceKinectOpen == false && req.active)
		ROS_INFO("Forcing Kinect open");
	else if (mForceKinectOpen && req.active == false)
		ROS_INFO("No longer forcing Kinect");

	mForceKinectOpen = req.active;
	return true;
}

/**
 * Service handler for enabling/disabling the forced generating of depth images (not publishing).
 */
bool KinectServer::ForceDepth(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (mForceDepth == false && req.active)
		ROS_INFO("Forcing depth open");
	else if (mForceDepth && req.active == false)
		ROS_INFO("No longer forcing depth");

	mForceDepth = req.active;
	return true;
}

/**
 * Service handler for retrieving a single point cloud. Assumes the kinect is generating depth.
 * Use /KinectServer/ForceDepth to force depth generating on.
 */
bool KinectServer::CloudServer(nero_msgs::GetCloud::Request &req, nero_msgs::GetCloud::Response &res)
{
	if (mKinect.isDepthGenerating() == false)
	{
		ROS_WARN("Received cloud request while not generating depth, starting depth now.");
		ROS_WARN("It is suggested to use the /KinectServer/ForceDepth service to force it open.");
		mKinect.startDepth();
	}

	mKinect.queryFrame(false, true);
	cv::Mat cloud = mKinect.getCloud(mKinect.getDepth());
	res.cloud = *OpenCVTools::matToPointCloud2(cloud);
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
