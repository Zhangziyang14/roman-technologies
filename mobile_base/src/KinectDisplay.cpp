/*
 * KinectDisplay.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: hans
 */

#include "Util.h"
#include "ros/ros.h"

IplImage *imageToSharedIplImage(sensor_msgs::ImagePtr image);

void imageCb(sensor_msgs::ImagePtr image)
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
	ros::Subscriber image_sub = n.subscribe("image", 1, imageCb);

	cv::startWindowThread();
	cvNamedWindow(WINDOW_NAME);
	ros::spin();

	return 0;
}
