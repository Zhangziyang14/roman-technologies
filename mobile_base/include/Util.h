#ifndef UTIL_H_
#define UTIL_H_

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"

#include "opencv/cxcore.h"  
#include "opencv/cvwimage.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>

#define WINDOW_NAME "KinectTest"
#define ROBOT_RADIUS 0.35f
#define EVADE_DISTANCE 700 // mm

typedef unsigned int uint32;
typedef unsigned short int uint16;
typedef unsigned char uint8;

template <class depthType>
inline depthType *getPixel(int x, int y, IplImage *image, int channel = 0)
{
	return &((depthType *)image->imageData)[(image->width * y + x) * image->nChannels + channel];
}
template <class depthType>
inline depthType *getPixel(cv::Point p, IplImage *image, int channel = 0) { return getPixel<depthType>(p.x, p.y, image, channel); };

inline uint16 getDepthFromRealPoint(cv::Point3f p)
{
	// pythagoras
	return uint16(1000 * sqrt(p.x*p.x + p.y*p.y + p.z*p.z));
}

inline cv::Point3f getPointFromCloud(int x, int y, IplImage *image)
{
	cv::Point3f p;
	p.x = *getPixel<float>(x, y, image, 0);
	p.y = *getPixel<float>(x, y, image, 1);
	p.z = *getPixel<float>(x, y, image, 2);
	return p;
}

inline uint16 getDepthFromCloud(int x, int y, IplImage *image)
{
	cv::Point3f p = getPointFromCloud(x, y, image);
	return getDepthFromRealPoint(p);
}
inline uint16 getDepthFromCloud(cv::Point p, IplImage *image) { return getDepthFromCloud(p.x, p.y, image); };

#endif
