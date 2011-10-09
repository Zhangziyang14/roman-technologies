#ifndef UTIL_H_
#define UTIL_H_

#include "sensor_msgs/LaserScan.h"

#include "opencv/cxcore.h"  
#include "opencv/cvwimage.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#define WINDOW_NAME "KinectTest"
#define ROBOT_RADIUS 0.35f
#define EVADE_DISTANCE 700 // mm

typedef unsigned int uint32;
typedef unsigned short int uint16;
typedef unsigned char uint8;

enum DisplayType
{
	DISPLAY_TYPE_RGB,
	DISPLAY_TYPE_GRAY,
	DISPLAY_TYPE_DEPTH,
	DISPLAY_TYPE_TOTAL,
};

inline uint16 *getPixelU16(int x, int y, IplImage *image, int channel = 0)
{
	return &((uint16 *)image->imageData)[(image->width * y + x) * image->nChannels + channel];
}
inline uint16 *getPixelU16(cv::Point p, IplImage *image, int channel = 0) { return getPixelU16(p.x, p.y, image, channel); };



inline uint8 *getPixelU8(int x, int y, IplImage *image, int channel = 0)
{
	return &((uint8 *)image->imageData)[(image->width * y + x) * image->nChannels + channel];
}
inline uint8 *getPixelU8(cv::Point p, IplImage *image, int channel = 0) { return getPixelU8(p.x, p.y, image, channel); };



inline bool *getPixelU1(int x, int y, IplImage *image, int channel = 0)
{
	return &((bool *)image->imageData)[(image->width * y + x) * image->nChannels + channel];
}
inline bool *getPixelU1(cv::Point p, IplImage *image, int channel = 0) { return getPixelU1(p.x, p.y, image, channel); };



inline float *getPixelF32(int x, int y, IplImage *image, int channel = 0)
{
	return &((float *)image->imageData)[(image->width * y + x) * image->nChannels + channel];
}
inline float *getPixelF32(cv::Point p, IplImage *image, int channel = 0) { return getPixelF32(p.x, p.y, image, channel); };


inline uint16 getDepthFromRealPoint(cv::Point3f p)
{
	// pythagoras
	return uint16(1000 * sqrt(p.x*p.x + p.y*p.y + p.z*p.z));
}


inline cv::Point3f getPointFromCloud(int x, int y, IplImage *image)
{
	cv::Point3f p;
	p.x = *getPixelF32(x, y, image, 0);
	p.y = *getPixelF32(x, y, image, 1);
	p.z = *getPixelF32(x, y, image, 2);
	return p;
}

inline uint16 getDepthFromCloud(int x, int y, IplImage *image)
{
	cv::Point3f p = getPointFromCloud(x, y, image);
	return getDepthFromRealPoint(p);
}
inline uint16 getDepthFromCloud(cv::Point p, IplImage *image) { return getDepthFromCloud(p.x, p.y, image); };

inline


#endif
