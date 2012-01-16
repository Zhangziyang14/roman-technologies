#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/distortion_models.h>

// forward declarations of some needed functions
sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud);
sensor_msgs::ImagePtr iplImageToImage(IplImage *image);
IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);
pcl::PointCloud<pcl::PointXYZ>::Ptr iplImageToPointCloud(IplImage *image);
sensor_msgs::PointCloud2Ptr iplImageToRegisteredPointCloud2(IplImage *pc, IplImage *rgb);

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
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);

	bool captureRGB = false;
	bool captureCloud = false;

	sensor_msgs::ImagePtr imageMsg;

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		//IplImage iplImage;

		// is it required to convert to point clouds and publish ?
		captureCloud = cloud_pub.getNumSubscribers() != 0;
		// is it required to capture RGB images ?
		captureRGB = image_pub.getNumSubscribers() != 0;

		if (capture->grab() == false)
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		// try to capture a RGB and pointcloud
		if (((captureRGB || captureCloud) && capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE)) &&
			(captureCloud == false || capture->retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP)))
		{
			IplImage rgb;
			if (captureRGB || captureCloud)
				rgb = image;
			IplImage pc = pointCloud;

			// SLAM / AMCL
			if (laser_pub.getNumSubscribers())
			{
				sensor_msgs::LaserScanPtr laserscan = iplImageToLaserScan(pc);
				if (laserscan)
					laser_pub.publish(laserscan);
			}

			if (captureRGB || captureCloud)
				imageMsg = iplImageToImage(&rgb);

			// do we have a listener to the RGB output ?
			if (captureRGB)
				image_pub.publish(imageMsg);

			if (captureCloud)
			{
				sensor_msgs::PointCloud2Ptr cloudMsg = iplImageToRegisteredPointCloud2(&pc, &rgb);
				cloud_pub.publish(cloudMsg);
				//roscom.processPointcloud(cloudMsg);
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
