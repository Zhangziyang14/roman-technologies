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

#include <iostream>
#include <stdio.h>

#include "image_processing/Util.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "head/PitchYaw.h"

#include "image_processing/SetActive.h"

// forward declaration
IplImage *imageToSharedIplImage(const sensor_msgs::ImageConstPtr &image);

#define STOP_SPEED_TOLERANCE 0.01

cv::CascadeClassifier gCascade;
ros::Publisher *kinect_motor_pub = NULL;
ros::Subscriber *image_sub = NULL;
ros::NodeHandle *nh = NULL;
head::PitchYaw gCurrentOrientation;
bool active = false;
bool gLock = false;

void headSpeedCb(const head::PitchYaw &msg)
{
	bool tmpLock = gLock;
	gLock = fabs(msg.pitch) > STOP_SPEED_TOLERANCE || fabs(msg.yaw) > STOP_SPEED_TOLERANCE;
	if (tmpLock != gLock)
	{
		if (gLock)
			ROS_INFO("Locking movement.");
		else
			ROS_INFO("Unlocking movement.");
	}
}

void headPositionCb(const head::PitchYaw &msg)
{
	gCurrentOrientation = msg;
}

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
    		head::PitchYaw msg;
    		double pitch = atan(p.y / p.z);
    		double yaw = -atan(p.x / p.z);

    		msg.pitch = pitch + gCurrentOrientation.pitch;
    		msg.yaw = yaw + gCurrentOrientation.yaw;
    		if (isnan(msg.pitch) || isnan(msg.yaw))
    			return;

    		kinect_motor_pub->publish(msg);
    	}
    }
    //cv::imshow("result", img);
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

	if (gLock)
	{
		//cvShowImage("result", iplImg);
		cvReleaseImage(&iplImg);
		return;
	}

	frame = iplImg;
	detectAndDraw(&cloud, frame, gCascade, /*nestedCascade, */ 2.0);

	cvReleaseImage(&iplImg);
}

bool setActiveCB(image_processing::SetActive::Request &req, image_processing::SetActive::Response &res)
{
	if (active && req.active == false)
	{
		image_sub->shutdown();
		delete image_sub;
		image_sub = NULL;
	}
	else if (active == false && req.active)
		image_sub = new ros::Subscriber(nh->subscribe("/camera/depth_registered/points", 1, &imageCb));

	active = req.active;
	return true;
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "FocusFace");
	nh = new ros::NodeHandle("");

	ros::Subscriber head_position_sub = nh->subscribe("/headPositionFeedbackTopic", 1, &headPositionCb);
	ros::Subscriber head_speed_sub = nh->subscribe("/headSpeedFeedbackTopic", 1, &headSpeedCb);
	kinect_motor_pub = new ros::Publisher(nh->advertise<head::PitchYaw>("/cmd_head_position", 1));
	ros::ServiceServer active_server = nh->advertiseService("/set_focus_face", &setActiveCB);

	gCurrentOrientation.pitch = 0.f;
	gCurrentOrientation.yaw = 0.f;

	if (argc != 2)
	{
		ROS_ERROR("Invalid input arguments.");
		return 0;
	}

	//cv::startWindowThread();
	//cvNamedWindow("result", 1);
    if (!gCascade.load(argv[1]))
    {
        std::cerr << "ERROR: Could not load classifier cascade" << std::endl;
        return -1;
    }

	int sleep_rate;
	nh->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		if (active == false)
			sleep.sleep();
		ros::spinOnce();
	}
}
