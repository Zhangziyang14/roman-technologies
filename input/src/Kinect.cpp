#include "Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#define PUSH_LASERSCAN_TIME (500*1000)

// forward declarations of some needed functions
sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud);
sensor_msgs::ImagePtr iplImageToImage(IplImage *image);
IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);
pcl::PointCloud<pcl::PointXYZ>::Ptr iplImageToPointCloud(IplImage *image);
sensor_msgs::PointCloud2Ptr iplImageToRegisteredPointCloud2(IplImage *pc, IplImage *rgb);

/**
 * Processes the pointcloud and display feedback on the RGB image.
 */
void processImage(IplImage *rgb, pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	// flip the images, moving right in front of the screen will move right on the screen
	//cvFlip(rgb, rgb, 1);
	//cvFlip(pc, pc, 1);

	// convert IplImage pointcloud to pcl::PointCloud for later algorithms
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = iplImageToPointCloud(pc);
	if (pc == NULL)
	{
		ROS_ERROR("Failed to make pcl::PointCloud.");
		return;
	}

	// center of the image
	cv::Point p = cvPoint(pc->width >> 1, pc->height >> 1);

	// find indices in the picture that are 'of interest'
	std::vector<int> indices;
	for (uint32 y = 0; y < pc->height; y++)
	{
		for (uint32 x = 0; x < pc->width; x++)
		{
			pcl::PointXYZ p = pc->at(x, y);
			int depth = getDepthFromPoint(p);
			if (depth == 0)	//) || depth > 3000)
				continue;

			if (p.x > -ROBOT_RADIUS &&
				p.x < ROBOT_RADIUS &&
				p.y < 0.f)
			{
				indices.push_back(pc->width * y + x);
			}
		}
	}

	// do we have interesting points?
	if (indices.size())
	{
		Eigen::Vector4f plane;
		float curvature;

		// try to fit a plane on these points
		pcl::computePointNormal<pcl::PointXYZ>(*pc, indices, plane, curvature);

		// all points above this fitted plane are considered obstacles and are colored red
		for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); it++)
		{
			float distance = getDistanceFromPointToPlane(plane, pc->at(*it));
			if (rgb)
				*getPixel<uint8>(*it, rgb, distance > 0.05f ? 2 : 0) = 255;
		}
	}
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture, ros::NodeHandle *n)
{
	bool quit = false;

	// this provides image compression
	image_transport::ImageTransport it(*n);
	ros::Publisher laser_pub = n->advertise<sensor_msgs::LaserScan>("scan", 1);
	//image_transport::Publisher image_pub = it.advertise("image", 1);
	ros::Publisher image_pub = n->advertise<sensor_msgs::Image>("/camera/rgb/image_color", 1);
	ros::Publisher cloud_pub = n->advertise<sensor_msgs::PointCloud2>("/camera/rgb/points", 1);
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


	long unsigned int laserTime = ros::Time::now().toNSec();
	bool captureRGB = false;
	bool captureCloud = false;

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		//IplImage iplImage;

		// is it required to convert to point clouds and publish ?
		captureCloud = cloud_pub.getNumSubscribers() != 0;
		// is it required to capture RGB images ?
		captureRGB = captureCloud || image_pub.getNumSubscribers() != 0;

		if (capture->grab() == false)
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		// try to capture a RGB and pointcloud
		if ((captureRGB == false || capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE)) &&
				capture->retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP))
		{
			IplImage rgb;
			if (captureRGB)
				rgb = image;
			IplImage pc = pointCloud;

			//pcl::PointCloud<pcl::PointXYZ>::Ptr pclpc = iplImageToPointCloud(&pc);
			//processImage(captureRGB ? &rgb : NULL, pclpc);

			// SLAM
			if (laser_pub.getNumSubscribers() && ros::Time::now().toNSec() >= laserTime)
			{
				sensor_msgs::LaserScanPtr laserscan = iplImageToLaserScan(pc);
				if (laserscan)
					laser_pub.publish(laserscan);
				laserTime = ros::Time::now().toNSec() + PUSH_LASERSCAN_TIME;
			}

			// do we have a listener to the RGB output ?
			if (captureRGB)
			{
				sensor_msgs::ImagePtr imageMsg = iplImageToImage(&rgb);
				image_pub.publish(imageMsg);
			}

			if (captureCloud)
			{
				sensor_msgs::PointCloud2Ptr cloudMsg = iplImageToRegisteredPointCloud2(&pc, &rgb);
				cloud_pub.publish(cloudMsg);
			}
		}

		// check for pressed keys
		int key = cv::waitKey(30);
		switch (key)
		{
		// Esc
		case 27:
			quit = true;
			break;
		default:
			break;
		}
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

	kinectLoop(&capture, &n);
	return 0;
}
