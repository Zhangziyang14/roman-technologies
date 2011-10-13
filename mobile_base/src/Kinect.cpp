#include "Util.h"
#include "ros/ros.h"

#define PUSH_LASERSCAN_TIME (500*1000)

sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud);
sensor_msgs::ImagePtr iplImageToImage(IplImage *image);
IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture, ros::NodeHandle *n)
{
	bool quit = false;
	long unsigned int laserTime = ros::Time::now().toNSec();
	ros::Publisher laser_pub = n->advertise<sensor_msgs::LaserScan>("scan", 1);
	ros::Publisher image_pub = n->advertise<sensor_msgs::Image>("image", 1);

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		//IplImage iplImage;

		if (!capture->grab())
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		if (capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE) && capture->retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP))
		{
			IplImage rgb = image;
			IplImage pc = pointCloud;
			cvFlip(&rgb, &rgb, 1);
			cvFlip(&pc, &pc, 1);

			cv::Point p = cvPoint(rgb.width >> 1, rgb.height >> 1);
			cvDrawCircle(&rgb, p, 5, cvScalar(0, 255, 0), 1, CV_AA);

			/*int minDepth = 9999;
			cv::Point closestPoint;
			for (int y = 0; y < pc.height; y++)
			{
				for (int x = 0; x < pc.width; x++)
				{
					int depth = getDepthFromCloud(x, y, &pc);
					if (depth == 0)
						continue;

					if (*getPixel<float>(x, y, &pc, 0) > -ROBOT_RADIUS &&
						*getPixel<float>(x, y, &pc, 0) < ROBOT_RADIUS &&
						*getPixel<float>(x, y, &pc, 1) < 0.f)
					{
						*getPixel<uint8>(x, y, &rgb, 0) = 255;
						if (depth < minDepth)
						{
							minDepth = depth;
							closestPoint = cvPoint(x, y);
						}
					}
				}
			}

			//uint16 depth = getDepthFromCloud(p, &pc);
			//std::cout << "Depth at center: " << depth << std::endl;
			if (minDepth != 9999)
			{
				cvDrawCircle(&rgb, closestPoint, 5, cvScalar(0, 0, 255), 1, CV_AA);
				std::cout << "Closest obstacle at :" << minDepth << std::endl;
			}*/

			if (ros::Time::now().toNSec() >= laserTime)
			{
				sensor_msgs::LaserScanPtr laserscan = iplImageToLaserScan(pc);
				if (laserscan)
					laser_pub.publish(laserscan);
				laserTime = ros::Time::now().toNSec() + PUSH_LASERSCAN_TIME;
			}

			//cvShowImage(WINDOW_NAME, &rgb);
			sensor_msgs::ImagePtr imageMsg = iplImageToImage(&rgb);
			image_pub.publish(imageMsg);
		}

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
	ros::init(argc, argv, "Kinect");

	ros::NodeHandle n;

	std::cout << "Kinect opening ..." << std::endl;
	cv::VideoCapture capture(CV_CAP_OPENNI);
	if( !capture.isOpened() )
	{
		std::cout << "Can not open a capture object." << std::endl;
		return -1;
	}
	std::cout << "done." << std::endl;

	capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default

	// Print some avalible Kinect settings.
	std::cout << "\nDepth generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FRAME_MAX_DEPTH\t" << capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm" << std::endl <<
			"FPS\t" << capture.get(CV_CAP_PROP_FPS) << std::endl;

	std::cout << "\nImage generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FPS\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS) << std::endl;

	//cvNamedWindow(WINDOW_NAME);
	kinectLoop(&capture, &n);

	return 0;
}
