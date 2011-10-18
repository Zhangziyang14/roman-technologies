#include "Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#define PUSH_LASERSCAN_TIME (500*1000)

sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud);
sensor_msgs::ImagePtr iplImageToImage(IplImage *image);
IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);
pcl::PointCloud<pcl::PointXYZ>::Ptr iplImageToPointCloud(IplImage *image);

void processImage(IplImage *rgb, IplImage *pc)
{
	cvFlip(rgb, rgb, 1);
	cvFlip(pc, pc, 1);

	cv::Point p = cvPoint(rgb->width >> 1, rgb->height >> 1);
	cvDrawCircle(rgb, p, 5, cvScalar(0, 255, 0), 1, CV_AA);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = iplImageToPointCloud(pc);
	if (pointcloud == NULL)
	{
		ROS_ERROR("Failed to make pcl::PointCloud.");
		return;
	}

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//pcl::PointCloud<pcl::Normal> normals;

	ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(pointcloud);
	//ne.compute(normals);

	std::cout << "1" << std::endl;
	//std::cout << "Point(" << p.x << ", " << p.y << "): x=" << pointcloud->points[pointcloud->width * p.y + p.x].x <<
	//		" y=" << pointcloud->points[pointcloud->width * p.y + p.x].y << " z=" << pointcloud->points[pointcloud->width * p.y + p.x].x << std::endl;
	//std::cout << "Depth at center: " << sqrt(pointcloud->at(pointcloud->width * p.y + p.x).x*pointcloud->at(pointcloud->width * p.y + p.x).x +
	//		pointcloud->at(pointcloud->width * p.y + p.x).y*pointcloud->at(pointcloud->width * p.y + p.x).y +
	//		pointcloud->at(pointcloud->width * p.y + p.x).z*pointcloud->at(pointcloud->width * p.y + p.x).z) << std::endl;

	/*pcl::PointXYZ p1, p2, p3, u, v, n;
	p1.x = *getPixel<float>(p.x, p.y, pc, 0);
	p1.y = *getPixel<float>(p.x, p.y, pc, 1);
	p1.z = *getPixel<float>(p.x, p.y, pc, 2);

	p2.x = *getPixel<float>(p.x-10, p.y, pc, 0);
	p2.y = *getPixel<float>(p.x-10, p.y, pc, 1);
	p2.z = *getPixel<float>(p.x-10, p.y, pc, 2);

	p3.x = *getPixel<float>(p.x+10, p.y+10, pc, 0);
	p3.y = *getPixel<float>(p.x+10, p.y+10, pc, 1);
	p3.z = *getPixel<float>(p.x+10, p.y+10, pc, 2);

	u.x = p2.x - p1.x;
	u.y = p2.y - p1.y;
	u.z = p2.z - p1.z;
	v.x = p3.x - p1.x;
	v.y = p3.y - p1.y;
	v.z = p3.z - p1.z;
	n.x = u.y*v.z - u.z*v.y;
	n.y = u.x*v.x - u.x*v.z;
	n.z = u.x*v.y - u.y*v.x;

	//std::cout << "normal: x= " << n.x << " y=" << n.y << " n.z=" << n.z << std::endl;
	float f = 1.f;
	if (n.x >= n.y && n.x >= n.z)
		f = n.x;
	else if (n.y >= n.x && n.x >= n.z)
		f = n.y;
	else if (n.z >= n.x && n.z >= n.y)
		f = n.z;

	if (f == 0.f)
	{
		f = 1.f;
		std::cout << "NO NORMALIZED VECTOR" << std::endl;
	}

	std::cout << "angle = " << acos(n.y / f) << std::endl;*/

	/*std::cout << "1" << std::endl;
	// Create a KD-Tree
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud(pointcloud);

	std::cout << "2" << std::endl;

	// Output has the same type as the input one, it will be only smoothed
	pcl::PointCloud<pcl::PointXYZ> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

	// Optionally, a pointer to a cloud can be provided, to be set by MLS
	pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	mls.setOutputNormals (mls_normals);

	std::cout << "3" << std::endl;

	// Set parameters
	mls.setInputCloud (pointcloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	std::cout << "4" << std::endl;

	// Reconstruct
	mls.reconstruct (mls_points);

	std::cout << "5" << std::endl;

	// Concatenate fields for saving
	pcl::PointCloud<pcl::PointNormal> mls_cloud;
	pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

	std::cout << "6" << std::endl;

	pcl::Normal n = mls_normals->at(p.x, p.y);*/
	pcl::Normal n;
	ne.computePointNormal(p.x, p.y, n);
	std::cout << "2" << std::endl;
	std::cout << "normal.x: " << n.normal[0] << " normal.y: " << n.normal[1] << " normal.z: " << n.normal[2] << std::endl;

	int minDepth = 9999;
	cv::Point closestPoint;
	for (int y = 0; y < pc->height; y++)
	{
		for (int x = 0; x < pc->width; x++)
		{
			int depth = getDepthFromCloud(x, y, pc);
			if (depth == 0)
				continue;

			if (*getPixel<float>(x, y, pc, 0) > -ROBOT_RADIUS &&
				*getPixel<float>(x, y, pc, 0) < ROBOT_RADIUS &&
				*getPixel<float>(x, y, pc, 1) < 0.f)
			{
				*getPixel<uint8>(x, y, rgb, 0) = 255;
				if (depth < minDepth)
				{
					minDepth = depth;
					closestPoint = cvPoint(x, y);
				}
			}
		}
	}

	if (minDepth != 9999)
	{
		cvDrawCircle(rgb, closestPoint, 5, cvScalar(0, 0, 255), 1, CV_AA);
		//std::cout << "Closest obstacle at : " << minDepth << std::endl;
	}
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture, ros::NodeHandle *n)
{
	bool quit = false;
	long unsigned int laserTime = ros::Time::now().toNSec();
	image_transport::ImageTransport it(*n);
	ros::Publisher laser_pub = n->advertise<sensor_msgs::LaserScan>("scan", 1);
	image_transport::Publisher image_pub = it.advertise("image", 1);

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

			processImage(&rgb, &pc);

			// SLAM
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

	//cvNamedWindow(WINDOW_NAME);
	kinectLoop(&capture, &n);

	return 0;
}
