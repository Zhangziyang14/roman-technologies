#ifndef UTIL_H_
#define UTIL_H_

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "opencv/cxcore.h"  
#include "opencv/cvwimage.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#define WINDOW_NAME "KinectTest"
#define ROBOT_RADIUS 0.30f
#define EVADE_DISTANCE 700 // mm

typedef unsigned int uint32;
typedef unsigned short int uint16;
typedef unsigned char uint8;

/// returns value at channel of the IplImage at index.
template <class depthType>
inline depthType *getPixel(int index, IplImage *image, int channel = 0)
{
	return &((depthType *)image->imageData)[index * image->nChannels + channel];
}
template <class depthType>
inline depthType *getPixel(int x, int y, IplImage *image, int channel = 0) { return getPixel<depthType>(image->width * y + x, image, channel); };
template <class depthType>
inline depthType *getPixel(cv::Point p, IplImage *image, int channel = 0) { return getPixel<depthType>(image->width * p.y + p.x, image, channel); };

/// Uses pythagoras to calculate depth from kinect to a point.
inline uint16 getDepthFromPoint(float x, float y, float z)
{
	// pythagoras
	return uint16(1000 * sqrt(x*x + y*y + z*z));
}
inline uint16 getDepthFromPoint(cv::Point3f p) { return getDepthFromPoint(p.x, p.y, p.z); }
inline uint16 getDepthFromPoint(pcl::PointXYZ p) { return getDepthFromPoint(p.x, p.y, p.z); }
inline uint16 getDepthFromPoint(pcl::PointXYZRGB p) { return getDepthFromPoint(p.x, p.y, p.z); }

/// Returns a point from a pointcloud
inline cv::Point3f getPointFromCloud(int x, int y, IplImage *image)
{
	cv::Point3f p;
	p.x = *getPixel<float>(x, y, image, 0);
	p.y = *getPixel<float>(x, y, image, 1);
	p.z = *getPixel<float>(x, y, image, 2);
	return p;
}

/// Calculates the depth from a pointcloud at a point
inline uint16 getDepthFromCloud(int x, int y, IplImage *image)
{
	return getDepthFromPoint(getPointFromCloud(x, y, image));
}
inline uint16 getDepthFromCloud(cv::Point p, IplImage *image) { return getDepthFromCloud(p.x, p.y, image); };
inline uint16 getDepthFromCloud(int x, int y, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) { return getDepthFromPoint(pc->at(x, y)); };
inline uint16 getDepthFromCloud(int x, int y, pcl::PointCloud<pcl::PointXYZRGB> *pc) { return getDepthFromPoint(pc->at(x, y)); };
inline uint16 getDepthFromCloud(cv::Point p, pcl::PointCloud<pcl::PointXYZRGB> *pc) { return getDepthFromPoint(pc->at(p.x, p.y)); };

/// Calculates the distance from point p to plane with the form of plane(0)*x + plane(1)*y + plane(2)*z + plane(3) = 0
inline float getDistanceFromPointToPlane(Eigen::Vector4f plane, pcl::PointXYZ p)
{
	float divisor = sqrt(plane(0)*plane(0) + plane(1)*plane(1) + plane(2)*plane(2));
	if (divisor == 0.f)
		return std::numeric_limits<float>::quiet_NaN();

	return (plane(0)*p.x + plane(1)*p.y + plane(2)*p.z + plane(3)) / divisor;
}

#endif
