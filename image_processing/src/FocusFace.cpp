/*
 * FocusFace.cpp
 *
 *  Created on: Dec 8, 2011
 *      Author: hans
 */

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "pcl/ros/conversions.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <stdio.h>

#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

// forward declaration
IplImage *imageToSharedIplImage(const sensor_msgs::ImageConstPtr &image);

#define CASCADE_NAME "cascade/haarcascade_frontalface_alt.xml"
#define PI 3.14159265

cv::CascadeClassifier gCascade;
ros::Publisher *kinect_motor_pub = NULL;
double gCurrentAngle = 0.0;

void detectAndDraw(pcl::PointCloud<pcl::PointXYZRGB> *cloud, cv::Mat& img, cv::CascadeClassifier& cascade, double scale)
{
    int i = 0;
    double t = 0;
    std::vector<cv::Rect> faces;
    const static cv::Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)};
    cv::Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.1, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE
        ,
        cv::Size(30, 30) );
    t = (double)cvGetTickCount() - t;
    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
    int minDepth = 9999;
    cv::Point minPoint;
    for( std::vector<cv::Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        cv::Mat smallImgROI;
        std::vector<cv::Rect> nestedObjects;
        cv::Point center;
        cv::Scalar color = colors[i%8];
        int radius;
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
        circle( img, center, radius, color, 3, 8, 0 );

        int depth = getDepthFromCloud(center, cloud);
        if (depth < minDepth)
        {
        	minDepth = depth;
        	minPoint = center;
        }
        /*if( nestedCascade.empty() )
            continue;
        smallImgROI = smallImg(*r);*/
        /*nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
            1.1, 2, 0
            //|CV_HAAR_FIND_BIGGEST_OBJECT
            //|CV_HAAR_DO_ROUGH_SEARCH
            //|CV_HAAR_DO_CANNY_PRUNING
            |CV_HAAR_SCALE_IMAGE
            ,
            cv::Size(30, 30) );
        for( std::vector<cv::Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
        {
            center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
            center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
            radius = cvRound((nr->width + nr->height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );
        }*/
    }
    if (minDepth != 9999)
    {
    	ROS_INFO("Closest face at (%d, %d) with distance %dmm", minPoint.x, minPoint.y, minDepth);
    	pcl::PointXYZRGB p = cloud->at(minPoint.x, minPoint.y);
    	if (p.z)
    	{
    		std_msgs::Float64 msg;
    		double degree = atan(p.y / p.z) * 180 / PI;

    		msg.data = degree + gCurrentAngle;
    		ROS_INFO("Rotate by : %lf, sending : %lf, currentAngle : %lf", degree, msg.data, gCurrentAngle);
    		gCurrentAngle = msg.data;
    		if (isnan(gCurrentAngle))
    			gCurrentAngle = 0.0;

    		kinect_motor_pub->publish(msg);
    		usleep(500*1000);

    	}
    }
    cv::imshow( "result", img );
}

/**
 * Receives RGB images and displays them on screen.
 */
void imageCb(const sensor_msgs::PointCloud2Ptr &image)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg(*image, cloud);

    sensor_msgs::ImagePtr image_(new sensor_msgs::Image);
    pcl::toROSMsg (cloud, *image_);

	cv::Mat frame;
	IplImage *iplImg = imageToSharedIplImage(image_);
	if (iplImg == NULL)
	{
		std::cerr << "Failed to create an IplImage." << std::endl;
		return;
	}

	frame = iplImg;
	detectAndDraw(&cloud, frame, gCascade, /*nestedCascade, */ 2.0);
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "FocusFace");
	ros::NodeHandle nh;

	ros::Subscriber image_sub = nh.subscribe("/camera/rgb/points", 1, &imageCb);
	kinect_motor_pub = new ros::Publisher(nh.advertise<std_msgs::Float64>("/tilt_angle", 1));

	cv::startWindowThread();
	cvNamedWindow( "result", 1 );
    if (!gCascade.load(CASCADE_NAME))
    {
        std::cerr << "ERROR: Could not load classifier cascade" << std::endl;
        return -1;
    }

	std_msgs::Float64 msg;
	msg.data = 0.0;
	kinect_motor_pub->publish(msg);

    ros::spin();
}
