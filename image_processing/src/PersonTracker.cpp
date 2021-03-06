/*
 * PersonTracker.cpp
 *
 *  Created on: May 22, 2012
 *      Author: hans
 */

#include "image_processing/PersonTracker.h"

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

		cv::imshow("PersonTracker", imgCopy);
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
PersonTracker::PersonTracker() :
	mTracking(false),
	mWaitForBBox(false),
	mBBox(0, 0, 0, 0),
	mCOG(0, 0),
	mLastDepth(0)
{
	// initialise services and topics
	mInitialPointSub		= mNodeHandle.subscribe("/PersonTracker/initial_point", 1, &PersonTracker::initialPointCb, this);
	mColorDepthImageSub 	= mNodeHandle.subscribe("/camera/color_depth", 1, &PersonTracker::imageColorDepthCb, this);
	mHeadSpeedSub 			= mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &PersonTracker::headSpeedCb, this);
	mTrackedPointPub 		= mNodeHandle.advertise<geometry_msgs::PointStamped>("/PersonTracker/point", 10);
	mHighPointPub 			= mNodeHandle.advertise<geometry_msgs::PointStamped>("/Head/follow_point", 10);

	mProjectClient			= mNodeHandle.serviceClient<nero_msgs::QueryCloud>("/KinectServer/ProjectPoints", true);
	mFocusFaceClient		= mNodeHandle.serviceClient<nero_msgs::SetActive>("/set_focus_face", true);

	mNodeHandle.param<bool>("/PersonTracker/show_output", mShowOutput, true);

	if (mShowOutput)
	{
		cv::startWindowThread();
		cv::namedWindow("PersonTracker", 1);
	}
}

void PersonTracker::headSpeedCb(const nero_msgs::PitchYaw &msg)
{
	mHeadSpeed = msg;
}

void PersonTracker::initialPointCb(const geometry_msgs::Point &point)
{
	if (mLastDepthImage.empty())
	{
		ROS_ERROR("Received initial point but without depth image.");
		return;
	}

	if (point.x < 0 || point.x >= mLastDepthImage.cols || point.y < 0 || point.y >= mLastDepthImage.rows)
	{
		ROS_ERROR("Initial point is out of image bounds.");
		return;
	}

	if (point.x == 0 && point.y == 0)
	{
		mTracking = false;
		geometry_msgs::PointStamped p;
		p.header.frame_id = "/base_link";
		p.point.x = 0.0;
		p.point.y = 0.0;
		p.point.z = 0.0;
		mTrackedPointPub.publish(p);
		//setFocusFace(true);
		return;
	}

	mLastDepth = mLastDepthImage.at<uint16_t>(point.y, point.x);
	if (mLastDepth > 0)
	{
		mCOG = cv::Point2i(point.x, point.y);
		mTracking = true;
		//setFocusFace(false);
	}
	else
		ROS_WARN("Depth at person follow point is 0.");
}

geometry_msgs::PointStamped PersonTracker::getWorldPoint(cv::Point2i p, const char *frame, uint16_t depth)
{
	nero_msgs::QueryCloud qc;
	geometry_msgs::Point gp;
	gp.x = p.x;
	gp.y = p.y;
	gp.z = depth ? depth : mLastDepthImage.at<uint16_t>(p.y, p.x);
	qc.request.points.push_back(gp);

	if (mProjectClient.call(qc) == false || qc.response.points.size() == 0)
	{
		ROS_WARN("Failed to call project cloud server.");
		return geometry_msgs::PointStamped();
	}

	geometry_msgs::PointStamped result;
	try
	{
		mTransformListener.waitForTransform(frame, qc.response.points[0].header.frame_id, qc.response.points[0].header.stamp, ros::Duration(1.0));
		mTransformListener.transformPoint(frame, qc.response.points[0], result);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		return geometry_msgs::PointStamped();
	}

	return result;
}

void PersonTracker::imageColorDepthCb(const nero_msgs::ColorDepthPtr &image)
{
	// dont refresh image when waiting for bounding box
	if (mWaitForBBox)
		return;

	mLastImage = OpenCVTools::imageToMat(image->color);
	mLastDepthImage = OpenCVTools::imageToMat(image->depth);

	if (mTracking)
	{
		if (abs(mLastDepthImage.at<uint16_t>(mCOG.y, mCOG.x) - mLastDepth) < ABSOLUTE_DEPTH_TOLERANCE)
		{
			cv::Mat person = cv::Mat::zeros(mLastDepthImage.rows, mLastDepthImage.cols, mLastDepthImage.type());
			cv::Point2i highPoint(0, 0);
			if (seedImage(mLastDepthImage, person, mCOG, mBBox, mCOG, highPoint))
			{
				geometry_msgs::PointStamped p = getWorldPoint(mCOG, "/base_link");
				mTrackedPointPub.publish(p);
				if (mShowOutput)
					cv::rectangle(mLastImage, mBBox, CV_RGB(0,0,255), 8, 8, 0);

				if (canMoveHead() && highPoint.x && highPoint.y)
				{
					geometry_msgs::PointStamped hp = getWorldPoint(highPoint, "/head_frame", mLastDepth);
					hp.point.z += HIGH_POINT_OFFSET;
					//mHighPointPub.publish(hp);
				}
			}
		}
	}

	if (mShowOutput)
	{
		cv::circle(mLastImage, mCOG, 3, CV_RGB(255, 0, 0), 3, 8);
		cv::imshow("PersonTracker", mLastImage);
	}
}

bool PersonTracker::seedImage(cv::Mat depth, cv::Mat &result, cv::Point2i seed, cv::Rect &r, cv::Point2i &cog, cv::Point2i &highPoint)
{
	std::queue<cv::Point2i> processQueue;
	processQueue.push(seed);

	int xmod[4] = { -1, 0, 0, 1 };
	int ymod[4] = { 0, -1, 1, 0 };

	cv::Point2i min(depth.cols - 1, depth.rows - 1);
	cv::Point2i max(0, 0);
	cv::Point2i hp(depth.cols, depth.rows);

	double depthSum = 0.0;
	cv::Point2f cogSum(0.f, 0.f);
	uint32_t count = 0;

	// while we have something to process
	while (processQueue.empty() == false)
	{
		cv::Point2i p = processQueue.front();
		processQueue.pop();

		uint16_t d = depth.at<uint16_t>(p.y, p.x);

		for (int i = 0; i < 4; i++)
		{
			cv::Point2i n(p.x + xmod[i], p.y + ymod[i]);

			if (n.x < 0 || n.y < 0 || n.x >= depth.cols || n.y >= depth.rows)
				continue;

			// is this neighbor close enough and not processed yet, then add it to be processed
			if (result.at<uint16_t>(n.y, n.x) == 0 &&
				abs(depth.at<uint16_t>(n.y, n.x) - d) < DEPTH_TOLERANCE)
			{
				// add it to the queue and set the result (mask) to the maximum value
				processQueue.push(n);
				result.at<uint16_t>(n.y, n.x) = uint16_t(-1);

				// update the bounding box
				min.x = std::min(min.x, n.x);
				min.y = std::min(min.y, n.y);
				max.x = std::max(max.x, n.x);
				max.y = std::max(max.y, n.y);

				// calculate mean depth
				depthSum += depth.at<uint16_t>(n.y, n.x);

				// calculate mean x,y
				cogSum.x += n.x;
				cogSum.y += n.y;

				if (n.y < hp.y)
					hp = n;

				count++;
			}
		}
	}

	if (count == 0)
		return false;

	r.x = std::max(0, min.x);
	r.y = std::max(0, min.y);
	r.width = std::min(depth.cols - r.x, max.x - r.x);
	r.height = std::min(depth.rows - r.y, max.y - r.y);

	cog.x = cogSum.x / count;
	cog.y = cogSum.y / count;

	mLastDepth = depthSum / count;

	if (hp.y < depth.rows)
		highPoint = hp;

	return true;
}

void PersonTracker::spin()
{
	while (ros::ok())
	{
		ros::spinOnce();

		if (mShowOutput)
		{
			switch (cv::waitKey(50) % 256)
			{
			// quit
			case 'q':
				return;
			// draw bounding box
			case 'r':
				if (mWaitForBBox && mLastImage.empty() == false)
					break;

				ROS_INFO("Capturing bounding box.");

				mWaitForBBox = true;
				bbox = cvRect(-1, -1, -1, -1);

				cv::setMouseCallback("PersonTracker", mouseCb, &mLastImage);
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

				cv::Mat grey;
				cv::cvtColor(mLastImage, grey, CV_BGR2GRAY);

				cv::setMouseCallback("PersonTracker", NULL, NULL);

				mBBox = bbox;
				mCOG = cv::Point2i(mBBox.x + (mBBox.width >> 1), mBBox.y + (mBBox.height >> 1));
				mLastDepth = mLastDepthImage.at<uint16_t>(mCOG.y, mCOG.x);
				mTracking = true;
				break;
			}
		}
	}
}

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "PersonTracker");

	PersonTracker pt;
	pt.spin();

	return 0;
}
