#include "image_server/KinectServer.h"

KinectServer::KinectServer() :
	mNodeHandle(""),
	mImageTransport(mNodeHandle),
	mPublishRGB(true),
	mPublishCloud(false),
	mKinect(CV_CAP_OPENNI)
{
	if (mKinect.isOpened())
		mKinect.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default
	
	mNodeHandle.param<bool>("/KinectServer/send_empty_laserscan", mSendEmptyLaserscan, false);
	
	mRGBPub = mImageTransport.advertise("/camera/rgb/image_color", 1);
	mPCPub = mNodeHandle.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
	mLaserPub = mNodeHandle.advertise<sensor_msgs::LaserScan>("/scan", 1);
	
	ROS_INFO("done.");
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void KinectServer::run()
{
	if (mKinect.isOpened() == false)
	{
		mKinect = cv::VideoCapture(CV_CAP_OPENNI);
		if (mKinect.isOpened() == false)
		{
			ROS_WARN("Can not open Kinect device, will try again in one second.");
			usleep(1000000);
			return;
		}
		else
			mKinect.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default
	}

	bool publishLaserScan = mSendEmptyLaserscan == false && mPublishLaserScan;
	bool captureRGB = mPublishRGB;
	bool captureCloud = mPublishCloud || publishLaserScan;
	
	cv::Mat rgb;
	if (captureRGB)
	{
		if (mKinect.retrieve(rgb, CV_CAP_OPENNI_BGR_IMAGE) == false)
		{
			ROS_WARN("Failed to retrieve RGB image.");
			return;
		}
		
		if (mPublishRGB)
			mRGBPub.publish(OpenCVTools::matToImage(rgb));
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
	
	/*bool quit = false;

	// this provides image compression
	image_transport::ImageTransport it(nh);
	ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
	image_transport::Publisher image_pub = it.advertise("/camera/rgb/image_color", 1);
	//ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_color", 1);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
	ros::Publisher rgbcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);

	bool captureRGB = false;
	bool captureCloud = false;
	bool publishRGBCloud = false;
	bool publishLaserscan = false;
	bool publishRGB = false;
	bool publishCloud = false;

	sensor_msgs::ImagePtr imageMsg;

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		//IplImage iplImage;

		// is it required to capture a laserscan ?
		publishLaserscan = gSendEmptyLaserscan == false && laser_pub.getNumSubscribers() != 0;
		// is it required to capture a pointcloud with rgb data ?
		publishRGBCloud = rgbcloud_pub.getNumSubscribers() != 0;
		// is it required to convert to point clouds and publish ?
		publishCloud = cloud_pub.getNumSubscribers() != 0;
		// is it required to capture RGB images ?
		publishRGB = image_pub.getNumSubscribers() != 0;
		// is it required to convert to point clouds and publish ?
		captureCloud = publishRGBCloud || publishLaserscan || publishCloud;
		// is it required to capture RGB images ?
		captureRGB = publishRGBCloud || publishRGB;

		if (capture->grab() == false)
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		// try to capture a RGB and pointcloud
		if ((captureRGB == false || capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE)) &&
			(captureCloud == false || capture->retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP)))
		{
			IplImage rgb;
			if (captureRGB)
				rgb = image;
			IplImage pc = pointCloud;

			// SLAM / AMCL
			if (publishLaserscan || gSendEmptyLaserscan)
			{
				sensor_msgs::LaserScanPtr laserscan = iplImageToLaserScan(&pc, gSendEmptyLaserscan);
				if (laserscan)
					laser_pub.publish(laserscan);
			}

			if (publishRGB)
			{
				imageMsg = iplImageToImage(&rgb);
				image_pub.publish(imageMsg);
			}

			if (publishCloud)
			{
				sensor_msgs::PointCloud2Ptr cloudMsg = iplImageToPointCloud2(&pc);
				cloud_pub.publish(cloudMsg);
			}

			if (publishRGBCloud)
			{
				sensor_msgs::PointCloud2Ptr cloudMsg = iplImageToRegisteredPointCloud2(&pc, &rgb);
				rgbcloud_pub.publish(cloudMsg);
			}
		}

		ros::spinOnce();
	}*/
}

int main(int argc, char* argv[])
{
	// init ros
	ros::init(argc, argv, "KinectServer");
	
	KinectServer kinectServer;

	while (ros::ok())
	{
		kinectServer.run();
		ros::spinOnce();
	}
	return 0;
}

