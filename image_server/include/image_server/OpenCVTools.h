/*
 * OpenCVTools.h
 *
 *  Created on: 2012-04-18
 *      Author: hgaiser
 */

#ifndef OPENCVTOOLS_H_
#define OPENCVTOOLS_H_

#include "ros/ros.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"

namespace OpenCVTools
{
	sensor_msgs::LaserScanPtr matToLaserScan(cv::Mat &cloud, bool emptyScan);
	cv::Mat imageToMat(const sensor_msgs::ImageConstPtr &image); //, cv::Mat &mat
	cv::Mat imageToMat(sensor_msgs::Image image); //, cv::Mat &mat
	sensor_msgs::ImagePtr matToImage(cv::Mat mat);
	sensor_msgs::PointCloud2Ptr matToPointCloud2(cv::Mat &mat);
	sensor_msgs::PointCloud2Ptr matToRegisteredPointCloud2(cv::Mat pc, cv::Mat rgb);
};

#endif /* OPENCVTOOLS_H_ */
