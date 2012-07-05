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

/**
 * converts from encoding string to OpenCV type id
 */
int encodingToId(std::string enc)
{
	// cannot unfortunately be in a switch
	if (enc == sensor_msgs::image_encodings::MONO8)
		return CV_8UC1;
	else if (enc == sensor_msgs::image_encodings::MONO16)
		return CV_16UC1;
	else if (enc == sensor_msgs::image_encodings::BGR8 ||
			  enc == sensor_msgs::image_encodings::RGB8)
		return CV_8UC3;
	else if (enc == sensor_msgs::image_encodings::BGRA8 ||
			  enc == sensor_msgs::image_encodings::RGBA8)
		return CV_8UC4;
	else if (enc == sensor_msgs::image_encodings::BGR16 ||
			  enc == sensor_msgs::image_encodings::RGB16)
		return CV_16UC3;
	else if (enc == sensor_msgs::image_encodings::BGRA16 ||
			  enc == sensor_msgs::image_encodings::RGBA16)
		return CV_16UC4;
	else
		return -1;
}

/**
 * converts from OpenCV type id to encoding string to
 */
std::string idToEncoding(int type)
{
	// cannot unfortunately be in a switch
	switch (type)
	{
	case CV_8UC1:
		return sensor_msgs::image_encodings::MONO8;
	case CV_16UC1:
		return sensor_msgs::image_encodings::MONO16;
	case CV_8UC3:
		return sensor_msgs::image_encodings::BGR8;
	case CV_8UC4:
		return sensor_msgs::image_encodings::BGRA8;
	case CV_16UC3:
		return sensor_msgs::image_encodings::BGR16;
	case CV_16UC4:
		return sensor_msgs::image_encodings::BGRA16;
	default:
		return "";
	}
}

/**
 * Converts sensor_msgs::Image to cv::Mat
 */
cv::Mat OpenCVTools::imageToMat(sensor_msgs::Image image) {
	int type = encodingToId(image.encoding);
	if (type == -1)
	{
		ROS_ERROR("[OpenCVTools] Invalid encoding specified: %s", image.encoding.c_str());
		return cv::Mat();
	}

	cv::Mat matTemp(image.height, image.width, type);
	memcpy(matTemp.data, &image.data[0], image.step * image.height);
	return matTemp;
}

/**
 * Converts sensor_msgs::ImageConstPtr to cv::Mat
 */
cv::Mat OpenCVTools::imageToMat(const sensor_msgs::ImageConstPtr &image)
{
	return imageToMat(*image);
}

/**
 * Converts a cv::Mat to sensor_msgs::ImagePtr
 */
sensor_msgs::ImagePtr OpenCVTools::matToImage(cv::Mat mat)
{
	sensor_msgs::ImagePtr output(new sensor_msgs::Image());

	// copy header
	output->header.stamp = ros::Time::now();
	output->width = mat.cols;
	output->height = mat.rows;
	output->step = mat.cols * mat.elemSize();
	output->is_bigendian = false;
	output->encoding = idToEncoding(mat.type());

	// copy actual data
	output->data.assign(mat.data, mat.data + size_t(mat.rows * output->step));
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
	output->point_step = 4*sizeof(float);
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

	output->data.resize(size_t(output->width * output->height * output->point_step));
	for (uint32_t x = 0; x < output->width; x++)
	{
		for (uint32_t y = 0; y < output->height; y++)
		{
			uint8_t data[16];
			cv::Point3f p = mat.at<cv::Point3f>(y, x);
			p.y = -p.y; // y-axis is inverted
			if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0)
				p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
			memcpy(&data[0], &p.x, sizeof(float));
			memcpy(&data[4], &p.y, sizeof(float));
			memcpy(&data[8], &p.z, sizeof(float));

			memcpy(&output->data[(y*output->width + x) * output->point_step], data, output->point_step);
		}
	}
	//output->data.assign(mat.data, mat.data + size_t(mat.cols * mat.rows * mat.channels()));
	return output;
}

/**
 * Converts cv::Mat to sensor_msgs::PointCloud2Ptr, with XYZRGB data
 */
sensor_msgs::PointCloud2Ptr OpenCVTools::matToRegisteredPointCloud2(cv::Mat pc, cv::Mat rgb)
{
	sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
	output->header.stamp = ros::Time::now();
	output->header.frame_id = OUTPUT_FRAME_ID;
	output->width = pc.cols;
	output->height = pc.rows;
	output->is_dense = false;
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
	pf.name = "_";
	pf.offset = 12;
	pf.count = 4;
	pf.datatype = sensor_msgs::PointField::UINT8;
	output->fields.push_back(pf);
	pf.name = "rgb";
	pf.offset = 16;
	pf.count = 1;
	pf.datatype = sensor_msgs::PointField::FLOAT32;
	output->fields.push_back(pf);
	pf.name = "_";
	pf.offset = 20;
	pf.count = 12;
	pf.datatype = sensor_msgs::PointField::UINT8;
	output->fields.push_back(pf);

	output->data.resize(size_t(pc.cols * pc.rows * output->point_step));
	for (int y = 0; y < pc.rows; y++)
	{
		for (int x = 0; x < pc.cols; x++)
		{
			uint8_t data[32];
			cv::Point3f p = pc.at<cv::Point3f>(y, x);
			p.y = -p.y; // y-axis is inverted
			if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0)
				p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
			memcpy(&data[0], &p.x, sizeof(cv::Point3f));

			cv::Point3_<uint8_t> c = rgb.at< cv::Point3_<uint8_t> >(y, x);
			memcpy(&data[16], &c, sizeof(cv::Point3_<uint8_t>));

			memcpy(&output->data[(y*pc.cols + x) * output->point_step], data, output->point_step);
		}
	}
	return output;
}

