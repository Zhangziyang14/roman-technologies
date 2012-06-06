/*
 * KinectDisplay.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: hans
 */

#include "image_server/OpenCVTools.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"

/**
 * Receives RGB images and displays them on screen.
 */
void imageCb(const sensor_msgs::ImageConstPtr &image)
{
	cv::Mat frame(image->height, image->width, CV_16UC1);
	OpenCVTools::imageToMat(image, frame);

	cv::imshow("Display", frame);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "KinectDisplay");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	//image_transport::TransportHints hints("compressed", ros::TransportHints(), n);
	image_transport::Subscriber image_sub = it.subscribe("/camera/depth/image", 1, &imageCb);

	cv::startWindowThread();
	cv::namedWindow("Display");
	ros::Rate rate(30);
	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
