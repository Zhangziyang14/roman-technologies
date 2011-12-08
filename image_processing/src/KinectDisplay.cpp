/*
 * KinectDisplay.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: hans
 */

#include "input/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

// forward declaration
IplImage *imageToSharedIplImage(const sensor_msgs::ImageConstPtr &image);

/**
 * Receives RGB images and displays them on screen.
 */
void imageCb(const sensor_msgs::ImageConstPtr &image)
{
	IplImage *iplImg = imageToSharedIplImage(image);
	if (iplImg)
	{
		cvShowImage(WINDOW_NAME, iplImg);
		cvReleaseImage(&iplImg);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "KinectDisplay");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_color", 1, &imageCb);

	cv::startWindowThread();
	cvNamedWindow(WINDOW_NAME);
	ros::spin();

	return 0;
}
