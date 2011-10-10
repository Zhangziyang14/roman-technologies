#include "Util.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#define MIN_HEIGHT 0.10
#define MAX_HEIGHT 0.15
#define ANGLE_MIN -M_PI_2
#define ANGLE_MAX M_PI_2
#define ANGLE_INCREMENT (M_PI/180.0/2.0)
#define SCAN_TIME (1.0/30.0)
#define RANGE_MIN 0.45
#define RANGE_MAX 10.0
#define OUTPUT_FRAME_ID "/openni_depth_frame"

#define PUSH_LASERSCAN_TIME (500*1000)

sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	ROS_INFO("Got cloud\n");

	//output->header = cloud->header;
	output->header.stamp = ros::Time::now();
	output->header.frame_id = OUTPUT_FRAME_ID;
	output->angle_min = ANGLE_MIN;
	output->angle_max = ANGLE_MAX;
	output->angle_increment = ANGLE_INCREMENT;
	output->time_increment = 0.0;
	output->scan_time = SCAN_TIME;
	output->range_min = RANGE_MIN;
	output->range_max = RANGE_MAX;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for (int row = 0; row < cloud.height; row++)
	{
		for (int col = 0; col < cloud.width; col++)
		{
			cv::Point3f p = getPointFromCloud(col, row, &cloud);

			if (p.x == 0 || p.y == 0 || p.z == 0)
			{
				//ROS_INFO("rejected for zero point");
				continue;
			}
			if ( std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) )
			{
				//ROS_INFO("rejected for nan in point(%f, %f, %f)\n", p.x, p.y, p.z);
				continue;
			}
			if (-p.y > MAX_HEIGHT || -p.y < MIN_HEIGHT)
			{
				//ROS_INFO("rejected for height %f not in range (%f, %f)\n", p.x, MIN_HEIGHT, MAX_HEIGHT);
				continue;
			}

			double angle = -atan2(p.x, p.z);
			if (angle < output->angle_min || angle > output->angle_max)
			{
				//ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
				continue;
			}
			int index = (angle - output->angle_min) / output->angle_increment;
			//printf ("index xyz( %f %f %f) angle %f index %d\n", x, y, z, angle, index);
			double range_sq = p.z*p.z + p.x*p.x;
			if (output->ranges[index] * output->ranges[index] > range_sq)
				output->ranges[index] = sqrt(range_sq);
		}
	}

	return output;
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture, ros::NodeHandle *n)
{
	bool quit = false;
	DisplayType displayType = DISPLAY_TYPE_DEPTH;
	long unsigned int laserTime = ros::Time::now().toNSec();
	ros::Publisher laser_pub = n->advertise<sensor_msgs::LaserScan>("scan", 1);

	while (quit == false && ros::ok())
	{
		cv::Mat image, pointCloud;
		IplImage iplImage;

		if (!capture->grab())
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		switch (displayType)
		{
		case DISPLAY_TYPE_RGB:
			if (capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE))
			{
				iplImage = image;
				cvShowImage(WINDOW_NAME, &iplImage);
			}
			break;
		case DISPLAY_TYPE_GRAY:
			if (capture->retrieve(image, CV_CAP_OPENNI_GRAY_IMAGE))
			{
				iplImage = image;
				cvShowImage(WINDOW_NAME, &iplImage);
			}
			break;
		case DISPLAY_TYPE_DEPTH:
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

						if (*getPixelF32(x, y, &pc, 0) > -ROBOT_RADIUS &&
							*getPixelF32(x, y, &pc, 0) < ROBOT_RADIUS &&
							*getPixelF32(x, y, &pc, 1) < 0.f)
						{
							*getPixelU8(x, y, &rgb, 0) = 255;
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
					laser_pub.publish(laserscan);

					laserTime = ros::Time::now().toNSec() + PUSH_LASERSCAN_TIME;
				}

				cvShowImage(WINDOW_NAME, &rgb);
			}
			break;
		default:
			break;
		}

		int key = cv::waitKey(30);
		switch (key)
		{
		// Esc
		case 27:
			quit = true;
			break;
		// Space
		case 32:
			displayType = DisplayType((displayType + 1) % DISPLAY_TYPE_TOTAL);
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

	cvNamedWindow(WINDOW_NAME);
	kinectLoop(&capture, &n);

	return 0;
}
