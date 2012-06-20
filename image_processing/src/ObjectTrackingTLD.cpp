/*
 * TLDObjectLearner.cpp
 *
 *  Created on: May 22, 2012
 *      Author: hans
 */

#include "image_processing/ObjectTrackingTLD.h"

static bool drag = false;
static cv::Point point;
static cv::Rect bbox;

static void mouseCb(int event, int x, int y, int flags, void* param)
{
	cv::Mat *img = (cv::Mat *)param;

	/* user press left button */
	if (event == CV_EVENT_LBUTTONDOWN && !drag) {
		point = cvPoint(x, y);
		drag = true;
	}

	/* user drag the mouse */
	if (event == CV_EVENT_MOUSEMOVE && drag) {
		cv::Mat imgCopy;
		img->copyTo(imgCopy);

		cv::rectangle(imgCopy, point, cvPoint(x, y), CV_RGB(255, 0, 0), 1, 8, 0);
		cv::imshow("ObjectTracking", imgCopy);
	}

	/* user release left button */
	if (event == CV_EVENT_LBUTTONUP && drag) {
		bbox = cvRect(point.x, point.y, x - point.x, y - point.y);
		drag = false;
	}
}

/**
 * Constructor
 */
//TODO: Add Setting the initial Bounding Box by subscribing to the topic
ObjectTrackingTLD::ObjectTrackingTLD() :
	mImageTransport(mNodeHandle),
	mTLD(new tld::TLD()),
	mWaitForBBox(false)
{
	// initialise services and topics
	image_transport::TransportHints hints("compressed", ros::TransportHints(), mNodeHandle);
	mImageSub 	= mImageTransport.subscribe("/camera/rgb/image_color", 1, &ObjectTrackingTLD::imageCb, this, hints);
	mObjPosPub 	= mNodeHandle.advertise<geometry_msgs::PointStamped>("/ObjectTracking/object_position", 1);

	mLoadModelServer 	= mNodeHandle.advertiseService("/ObjectTracking/load_model", &ObjectTrackingTLD::loadModelCb, this);
	mSetTrackingServer	= mNodeHandle.advertiseService("/ObjectTracking/set_tracking", &ObjectTrackingTLD::setTrackingCb, this);
	mSetLearningServer	= mNodeHandle.advertiseService("/ObjectTracking/set_learning", &ObjectTrackingTLD::setLearningCb, this);
	mSetDetectingServer	= mNodeHandle.advertiseService("/ObjectTracking/set_detecting", &ObjectTrackingTLD::setDetectingCb, this);

	// initialise params
	mNodeHandle.param<std::string>("/ObjectTracking/model_path", mModelPath, "/tmp/model.tld");

	cv::startWindowThread();
	cv::namedWindow("ObjectTracking", 1);

	// default image size
	mTLD->detectorCascade->imgWidth = 640;
	mTLD->detectorCascade->imgHeight = 480;
	mTLD->detectorCascade->imgWidthStep = 640;

	// show output colors
	mYellow = CV_RGB(255,255,0);
	mBlue = CV_RGB(0,0,255);
}

void ObjectTrackingTLD::sendPosition(cv::Rect r)
{
	geometry_msgs::PointStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "/rgb_frame";
	ps.point.x = double(r.x + r.width / 2) / mTLD->detectorCascade->imgWidth;
	ps.point.y = double(r.y + r.height / 2) / mTLD->detectorCascade->imgHeight;

	mObjPosPub.publish(ps);
}

void ObjectTrackingTLD::imageCb(const sensor_msgs::ImageConstPtr &image)
{
	// dont refresh image when waiting for bounding box
	if (mWaitForBBox)
		return;

	mLastImage = OpenCVTools::imageToMat(image);

	// set the TLD image size (needed when loading a model in TLD)
	mTLD->detectorCascade->imgWidth = mLastImage.cols;
	mTLD->detectorCascade->imgHeight = mLastImage.rows;
	mTLD->detectorCascade->imgWidthStep = mLastImage.step / 3; // divide by channel count, since it is about grayscale here

	// let TLD process the actual frame
	mTLD->processImage(mLastImage);

	if(mTLD->currBB != NULL)
	{
		cv::Scalar rectangleColor = (mTLD->currConf > 0.5) ? mBlue : mYellow;
		cv::rectangle(mLastImage, mTLD->currBB->tl(), mTLD->currBB->br(), rectangleColor, 8, 8, 0);
		sendPosition(*mTLD->currBB);
	}

	cv::imshow("ObjectTracking", mLastImage);
}

void ObjectTrackingTLD::spin()
{
	while (ros::ok())
	{
		ros::spinOnce();
		switch (cv::waitKey(50) % 256)
		{
		case 'c':
			mTLD->release();
			ROS_INFO("Cleared model from TLD.");
			break;
		// quit
		case 'q':
			return;
		// save
		case 's':
			mTLD->writeToFile(mModelPath.c_str());
			ROS_INFO("Saving model to file %s.", mModelPath.c_str());
			break;
		// draw bounding box
		case 'r':
			if (mWaitForBBox && mLastImage.empty() == false)
				break;

			ROS_INFO("Capturing bounding box.");

			mWaitForBBox = true;
			bbox = cvRect(-1, -1, -1, -1);

			cv::setMouseCallback("ObjectTracking", mouseCb, &mLastImage);
			break;
		// end drawing bounding box (Enter)
		case 13:
		case 10:
			if (mWaitForBBox == false)
				break;

			ROS_INFO("No longer waiting for bounding box.");
			mWaitForBBox = false;

			if (bbox.x == -1 || bbox.y == -1 || bbox.width == -1 || bbox.height == -1)
			{
				ROS_WARN("Invalid bounding box given.");
				break;
			}

			mTLD->selectObject(mLastImage, &bbox);
			break;
		}
	}
}

/**
 * Service callback for loading a model in TLD
 */
bool ObjectTrackingTLD::loadModelCb(nero_msgs::ObjectModel::Request &req, nero_msgs::ObjectModel::Response &res)
{
	mObjectModel = req;

	if (req.modelPath == "")
	{
		mTLD->release();
		ROS_INFO("Cleared model from TLD.");
	}
	else
	{
		ROS_INFO("Loading file from: %s", req.modelPath.c_str());
		mTLD->readFromFile(req.modelPath.c_str());
	}

	return true;
}

/**
 * Service callback for setting the tracking on or off
 */
bool ObjectTrackingTLD::setTrackingCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (req.active != mTLD->trackerEnabled)
		ROS_INFO("%s tracking in TLD.", req.active ? "Enabling" : "Disabling");
	mTLD->trackerEnabled = req.active;
	return true;
}

/**
 * Service callback for setting the learning on or off
 */
bool ObjectTrackingTLD::setLearningCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (req.active != mTLD->learningEnabled)
		ROS_INFO("%s learning in TLD.", req.active ? "Enabling" : "Disabling");
	mTLD->learningEnabled = req.active;
	return true;
}

/**
 * Service callback for setting the detection on or off
 */
bool ObjectTrackingTLD::setDetectingCb(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	if (req.active != mTLD->detectorEnabled)
		ROS_INFO("%s detecting in TLD.", req.active ? "Enabling" : "Disabling");
	mTLD->detectorEnabled = req.active;
	return true;
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "ObjectTrackingTLD");
	srand(0);

	ObjectTrackingTLD ot;
	ot.spin();

	return 0;
}
