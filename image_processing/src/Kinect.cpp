#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

// forward declarations of some needed functions
sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud);
sensor_msgs::ImagePtr iplImageToImage(IplImage *image);
IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);
pcl::PointCloud<pcl::PointXYZ>::Ptr iplImageToPointCloud(IplImage *image);
sensor_msgs::PointCloud2Ptr iplImageToRegisteredPointCloud2(IplImage *pc, IplImage *rgb);
sensor_msgs::PointCloud2Ptr iplImageToPointCloud2(IplImage *image);

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture, ros::NodeHandle &nh)
{
	bool quit = false;

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

	int sleep_rate;
	nh.param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		//IplImage iplImage;

		// is it required to capture a laserscan ?
		publishLaserscan = laser_pub.getNumSubscribers() != 0;
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
			if (publishLaserscan)
			{
				sensor_msgs::LaserScanPtr laserscan = iplImageToLaserScan(pc);
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

		sleep.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char* argv[])
{
	// init ros
	ros::init(argc, argv, "Kinect");
	ros::NodeHandle n;

	// init kinect
	std::cout << "Kinect opening ..." << std::endl;
	cv::VideoCapture capture(CV_CAP_OPENNI);
	if( !capture.isOpened() )
	{
		std::cout << "Can not open a capture object." << std::endl;
		return -1;
	}
	std::cout << "done." << std::endl;

	capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default

	// Print some available Kinect settings.
	std::cout << "\nDepth generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FRAME_MAX_DEPTH\t" << capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm" << std::endl <<
			"FPS\t" << capture.get(CV_CAP_PROP_FPS) << std::endl;

	std::cout << "\nImage generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FPS\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS) << std::endl;

	kinectLoop(&capture, n);
	return 0;
}
