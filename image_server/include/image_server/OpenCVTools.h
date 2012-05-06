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

class OpenCVTools
{
public:
	static sensor_msgs::LaserScanPtr matToLaserScan(cv::Mat &cloud, bool emptyScan);
	static void imageToMat(const sensor_msgs::ImageConstPtr &image, cv::Mat &mat);
	static sensor_msgs::ImagePtr matToImage(cv::Mat mat);
	static sensor_msgs::PointCloud2Ptr matToPointCloud2(cv::Mat &mat);
	
	//static float getDistanceFromPointToPlane(Eigen::Vector4f plane, pcl::PointXYZ p);	

	/// Uses pythagoras to calculate depth from kinect to a point.
	inline static uint16_t getDepthFromPoint(float x, float y, float z)
	{
		// pythagoras
		return uint16_t(1000 * sqrt(x*x + y*y + z*z));
	}
	inline static uint16_t getDepthFromPoint(cv::Point3f p) { return getDepthFromPoint(p.x, p.y, p.z); }
	
	/*inline uint16 getDepthFromPoint(pcl::PointXYZ p) { return getDepthFromPoint(p.x, p.y, p.z); }
	inline uint16 getDepthFromPoint(pcl::PointXYZRGB p) { return getDepthFromPoint(p.x, p.y, p.z); }*/

	inline static uint16_t getDepthFromCloud(int x, int y, cv::Mat mat) { return getDepthFromPoint(mat.at<cv::Point3f>(y, x)); };
	/*inline static uint16 getDepthFromCloud(int x, int y, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) { return getDepthFromPoint(pc->at(x, y)); };
	inline static uint16 getDepthFromCloud(int x, int y, pcl::PointCloud<pcl::PointXYZRGB> *pc) { return getDepthFromPoint(pc->at(x, y)); };
	inline static uint16 getDepthFromCloud(cv::Point p, pcl::PointCloud<pcl::PointXYZRGB> *pc) { return getDepthFromPoint(pc->at(p.x, p.y)); };*/

};

#endif /* OPENCVTOOLS_H_ */
