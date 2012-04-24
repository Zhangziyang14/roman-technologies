#include "image_server/OpenCVTools.h"

// LaserScan parameters
#define MIN_HEIGHT 0.10
#define MAX_HEIGHT 0.15
#define ANGLE_MIN -M_PI_2
#define ANGLE_MAX M_PI_2
#define ANGLE_INCREMENT (M_PI/180.0/2.0)
#define SCAN_TIME (1.0/30.0)
#define RANGE_MIN 0.45
#define RANGE_MAX 10.0
#define OUTPUT_FRAME_ID "/kinect_frame"

/**
 * Converts an IplImage to a LaserScan. Based on pointcloud_to_laserscan package.
 */
sensor_msgs::LaserScanPtr OpenCVTools::matToLaserScan(cv::Mat &cloud, bool emptyScan)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());

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

	if (emptyScan)
		return output;
		
	for (int row = 0; row < cloud.rows; row++)
	{
		for (int col = 0; col < cloud.cols; col++)
		{
			cv::Point3f p = cloud.at<cv::Point3f>(row, col);

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

void OpenCVTools::imageToMat(const sensor_msgs::ImageConstPtr &image, cv::Mat &mat)
{
	memcpy(mat.data, &image->data[0], sizeof(uchar) * image->data.size());
}

sensor_msgs::ImagePtr OpenCVTools::matToImage(cv::Mat mat)
{
	sensor_msgs::ImagePtr output(new sensor_msgs::Image());
	if (mat.type() != CV_8UC3)
	{
		ROS_WARN("Cannot convert non-uint8 to sensor_msgs::Image. Depth = %d", mat.type());
		return output;
	}

	// copy header
	output->header.stamp = ros::Time::now();
	output->width = mat.cols;
	output->height = mat.rows;
	output->step = mat.cols * mat.channels();
	output->is_bigendian = false;
	output->encoding = "bgr8";

	// copy actual data
	output->data.assign(mat.data, mat.data + size_t(mat.cols * mat.rows * mat.channels()));
	return output;
}

/**
 * Converts IplImage's to sensor_msgs::PointCloud2Ptr
 */
sensor_msgs::PointCloud2Ptr OpenCVTools::matToPointCloud2(cv::Mat &mat)
{
        sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
        output->header.stamp = ros::Time::now();
        output->header.frame_id = OUTPUT_FRAME_ID;
        output->width = mat.cols;
        output->height = mat.rows;
        output->is_dense = false;
        output->point_step = 16;
        output->is_bigendian = false;
        output->point_step = 8*sizeof(float);
        output->row_step = output->width * output->point_step;


        sensor_msgs::PointField pf;
        pf.name = "x";
        pf.offset = 0;
        pf.count = 1;
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        output->fields.push_back(pf);
        pf.name = "y";
        pf.offset = 4;
        pf.count = 1;
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        output->fields.push_back(pf);
        pf.name = "z";
        pf.offset = 8;
        pf.count = 1;
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        output->fields.push_back(pf);


        output->data.assign(mat.data, mat.data + size_t(output->width * output->height * mat.depth()));
        return output;
}

/// Calculates the distance from point p to plane with the form of plane(0)*x + plane(1)*y + plane(2)*z + plane(3) = 0
/*static float OpenCVTools::getDistanceFromPointToPlane(Eigen::Vector4f plane, pcl::PointXYZ p);
{
	float divisor = sqrt(plane(0)*plane(0) + plane(1)*plane(1) + plane(2)*plane(2));
	if (divisor == 0.f)
		return std::numeric_limits<float>::quiet_NaN();

	return (plane(0)*p.x + plane(1)*p.y + plane(2)*p.z + plane(3)) / divisor;
}*/
