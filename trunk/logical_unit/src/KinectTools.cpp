/*
 * KinectTools.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: hans
 */

#include "Util.h"

// LaserScan parameters
#define MIN_HEIGHT 0.10
#define MAX_HEIGHT 0.15
#define ANGLE_MIN -M_PI_2
#define ANGLE_MAX M_PI_2
#define ANGLE_INCREMENT (M_PI/180.0/2.0)
#define SCAN_TIME (1.0/30.0)
#define RANGE_MIN 0.45
#define RANGE_MAX 10.0
#define OUTPUT_FRAME_ID "/openni_depth_frame"

/**
 * Converts an IplImage to a LaserScan. Based on pointcloud_to_laserscan package.
 */
sensor_msgs::LaserScanPtr iplImageToLaserScan(IplImage &cloud)
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
 * Converts IplImage's to ros images. Needed for serialization, so it can be sent over the network.
 */
sensor_msgs::ImagePtr iplImageToImage(IplImage *image)
{
	sensor_msgs::ImagePtr output(new sensor_msgs::Image());
	if (image->depth != IPL_DEPTH_8U)
	{
		ROS_WARN("Cannot convert non-uint8 to sensor_msgs::Image. Depth = %d", image->depth);
		return output;
	}

	// copy header
	output->header.stamp = ros::Time::now();
	output->width = image->width;
	output->height = image->height;
	output->step = image->width * image->nChannels;
	output->is_bigendian = false;
	output->encoding = "bgr8";

	// copy actual data
	output->data.assign(image->imageData, image->imageData + size_t(image->width * image->height * image->nChannels));
	return output;
}

/**
 * Converts ros images back to IplImage's. The memory of the ros image is the same as in the IplImage.
 */
IplImage *imageToSharedIplImage(const sensor_msgs::ImageConstPtr &image)
{
	if (image->width == 0)
	{
		ROS_ERROR("Image width is 0.");
		return NULL;
	}

	IplImage *output = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, image->step / (image->width));
	output->imageData = (char *)&image->data[0];
	return output;
}

/**
 * Converts IplImage's to pcl::PointCloud. Needed for PCL algorithms.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr iplImageToPointCloud(IplImage *image)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	output->header.stamp = ros::Time::now();
	output->width = image->width;
	output->height = image->height;
	output->is_dense = false;
	output->points.resize(output->width * output->height);

	pcl::PointCloud<pcl::PointXYZ>::iterator it = output->begin();
	for (uint32 y = 0; y < output->height; y++)
	{
		for (uint32 x = 0; x < output->width; x++, it++)
		{
			pcl::PointXYZ &p = *it;
			if (*getPixel<float>(x, y, image, 2) == 0.f)
			{
				p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
				continue;
			}

			p.x = *getPixel<float>(x, y, image, 0);
			p.y = *getPixel<float>(x, y, image, 1);
			p.z = *getPixel<float>(x, y, image, 2);
		}
	}
	return output;
}

/**
 * Converts IplImage's to sensor_msgs::PointCloud2Ptr
 */
sensor_msgs::PointCloud2Ptr iplImageToPointCloud2(IplImage *image)
{
	sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
	output->header.stamp = ros::Time::now();
	output->width = image->width;
	output->height = image->height;
	output->is_dense = false;
	//output->

	output->data.assign(image->imageData, image->imageData + size_t(image->width * image->height * image->nChannels));
	return output;
}
