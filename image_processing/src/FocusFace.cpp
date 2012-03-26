/*
 * FocusFace.cpp
 *
 *  Created on: Dec 8, 2011
 *      Author: hans
 */

#include "image_processing/FocusFace.h"

// forward declaration
void imageToMat(const sensor_msgs::ImageConstPtr &image, cv::Mat &mat);

cv::Point2f calcCenterOfGravity(std::vector<cv::Point2f> f)
{
	uint32_t size = f.size();
	cv::Point2f result;

	for (size_t i = 0; i < size; i++)
	{
		result.x += f[i].x;
		result.y += f[i].y;
	}

	result.x /= size;
	result.y /= size;
	return result;
}

float calcMeanSquareError(std::vector<cv::Point2f> f, cv::Point2f cog)
{
	uint32_t size = f.size();
	float mean_se = 0.f;
	for (size_t i = 0; i < size; i++)
	{
		float dx = f[i].x - cog.x;
		float dy = f[i].y - cog.y;
		mean_se += dx*dx + dy*dy;
	}

	return mean_se / size;
}

void pruneFeatures(std::vector<cv::Point2f> &f)
{
	cv::Point2f cog = calcCenterOfGravity(f);
	float mean_se = calcMeanSquareError(f, cog);

	// this would go wrong
	if (mean_se == 0.f)
		return;

	// throw away outliers
	size_t k = 0;
	for (size_t i = 0; i < f.size(); i++)
	{
		float dx = f[i].x - cog.x;
		float dy = f[i].y - cog.y;

		float se = (dx*dx + dy*dy) / mean_se;

		if (se <= OUTLIER_THRESHOLD)
			f[k++] = f[i];
	}
	f.resize(k);
}

FocusFace::FocusFace(const char *frontal_face, const char *profile_face, const char *frontal_face_2) :
		mStartTime(0)
{
	mHeadPosSub = mNodeHandle.subscribe("/headPositionFeedbackTopic", 1, &FocusFace::headPositionCb, this);
	mHeadSpeedSub = mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &FocusFace::headSpeedCb, this);
	mHeadPosPub = mNodeHandle.advertise<nero_msgs::PitchYaw>("/cmd_head_position", 1);
	mActiveServer = mNodeHandle.advertiseService("/set_focus_face", &FocusFace::setActiveCB, this);

	mCurrentOrientation.pitch = 0.f;
	mCurrentOrientation.yaw = 0.f;
	mActive = false;

    if (mFrontalFaceCascade.load(frontal_face)	 == false)
        ROS_ERROR("Could not load frontal face classifier cascade");
    if (profile_face == NULL || mProfileCascade.load(profile_face) == false)
    	ROS_WARN("Could not load profile face classifier cascade");
    if (profile_face == NULL || mFrontalFace2Cascade.load(frontal_face_2) == false)
    	ROS_WARN("Could not load frontal face2 classifier cascade");

    mNodeHandle.param<double>("image_scale", mScale, 2.0);
    mNodeHandle.param<bool>("display_frames", mDisplayFrames, true);

#ifdef WEBCAM
    mImageSub = mNodeHandle.subscribe("/camera/rgb/image_color", 1, &FocusFace::imageCb, this);
#endif

    if (mDisplayFrames)
    {
//		cv::startWindowThread();
//		cvNamedWindow("FaceFocus", 1);
    }

    ROS_INFO("FocusFace initialised.");
}

void FocusFace::headSpeedCb(const nero_msgs::PitchYaw &msg)
{
	mCurrentSpeed = msg;
}

void FocusFace::headPositionCb(const nero_msgs::PitchYaw &msg)
{
	mCurrentOrientation = msg;
}

void FocusFace::sendHeadPosition(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
	if (canMoveHead() == false)
		return;

    nero_msgs::PitchYaw msg;

    pcl::PointXYZRGB p = cloud.at(mFaceCenter.x, mFaceCenter.y);
    if (p.z == 0.f)
    {
    	ROS_ERROR("Invalid face center.");
    	return;
    }

    double pitch = atan(p.y / p.z);
    double yaw = -atan(p.x / p.z);

    msg.pitch = pitch + mCurrentOrientation.pitch;
    msg.yaw = yaw + mCurrentOrientation.yaw;
    if (isnan(msg.pitch) || isnan(msg.yaw))
            return;

    mHeadPosPub.publish(msg);
}

void FocusFace::detectFaces(cv::Mat &frame, pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
	uint16_t minDepth = -1;
	size_t minIndex = -1;
	cv::Mat gray_frame, scaled_frame;
	cv::cvtColor(frame, gray_frame, CV_BGR2GRAY);
	cv::equalizeHist(gray_frame, gray_frame);
	cv::resize(gray_frame, scaled_frame, cv::Size(gray_frame.cols / mScale, gray_frame.rows / mScale), 0, 0, cv::INTER_LINEAR);

	std::vector<cv::Rect> faces;
	mFrontalFaceCascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

	if (faces.size() == 0 && mProfileCascade.empty() == false)
		mProfileCascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
	if (faces.size() == 0 && mFrontalFace2Cascade.empty() == false)
		mFrontalFace2Cascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    for (size_t i = 0; i < faces.size(); i++)
    {
    	faces[i].x *= mScale;
    	faces[i].y *= mScale;
    	faces[i].height *= mScale;
    	faces[i].width *= mScale;

    	cv::rectangle(frame, faces[i], CV_RGB(255, 0, 0), 3);

#ifdef WEBCAM
		minDepth = 0;
		minIndex = i;
		break;
#endif

    	// look for the closest face
    	uint16_t depth = getDepthFromCloud(faces[i].x + (faces[i].width >> 1), faces[i].y + (faces[i].height >> 1), &cloud);
    	if (depth < minDepth)
    	{
    		minDepth = depth;
    		minIndex = i;
    	}
    }

    if (minDepth != uint16_t(-1))
    {
    	int radius = 0.25*(faces[minIndex].width + faces[minIndex].height);
    	mFaceCenter = cv::Point(faces[minIndex].x + faces[minIndex].width*0.5, faces[minIndex].y + faces[minIndex].height*0.5);
    	cv::Mat mask = cv::Mat::zeros(cv::Size(gray_frame.cols, gray_frame.rows), CV_8UC1);

    	cv::circle(mask, mFaceCenter, radius, cv::Scalar(255), CV_FILLED);

    	mFeatures[0].clear();
        cv::goodFeaturesToTrack(gray_frame, mFeatures[0], MAX_CORNERS, 0.01, 8, mask);

        if (mFeatures[0].size() > MIN_FEATURE_COUNT)
        {
        	if (mDisplayFrames)
        	{
				cv::Point2f cog = calcCenterOfGravity(mFeatures[0]);
				float mse = calcMeanSquareError(mFeatures[0], cog);
				cv::circle(frame, mFaceCenter, sqrtf(mse), CV_RGB(0, 120, 255), 3, CV_AA);

#ifndef WEBCAM
				sendHeadPosition(cloud);
#endif

				//for (size_t j = 0; j < mFeatures[0].size(); j++)
				//	cv::circle(frame, mFeatures[0][j], 3, CV_RGB(0, 125, 125), 1, CV_AA);
        	}
        }
        else
        	mFeatures[0].clear();
    }

    //if (mDisplayFrames)
    //	cv::imshow("FaceFocus", frame);

}

void FocusFace::trackFace(cv::Mat &prevFrame, cv::Mat &frame, pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
	if (prevFrame.empty())
		frame.copyTo(prevFrame);

	cv::Mat grayFrame[2];
	cv::cvtColor(prevFrame, grayFrame[0], CV_BGR2GRAY);
	cv::cvtColor(frame, grayFrame[1], CV_BGR2GRAY);

	cv::equalizeHist(grayFrame[0], grayFrame[0]);
	cv::equalizeHist(grayFrame[1], grayFrame[1]);

    std::vector<uchar> status;
    std::vector<float> err;
	cv::calcOpticalFlowPyrLK(grayFrame[0], grayFrame[1], mFeatures[0], mFeatures[1], status, err, cv::Size(10, 10), 3,
			cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.01));

	cv::Point min(frame.cols, frame.rows);
	cv::Point max(0, 0);
    size_t i, k;
    cv::Point2f diff;
    for (i = k = 0; i < mFeatures[1].size(); i++)
    {
        if (!status[i])
            continue;

        mFeatures[1][k++] = mFeatures[1][i];

        diff.x += mFeatures[1][i].x - mFeatures[0][i].x;
        diff.y += mFeatures[1][i].y - mFeatures[0][i].y;
    }
    mFeatures[1].resize(k);
    diff.x /= k;
    diff.y /= k;
    mFaceCenter.x += diff.x;
    mFaceCenter.y += diff.y;
    mFaceCenter.x = std::min(std::max(mFaceCenter.x, 0.f), float(frame.cols));
    mFaceCenter.y = std::min(std::max(mFaceCenter.y, 0.f), float(frame.rows));

    pruneFeatures(mFeatures[1]);
    if (mFeatures[1].size() > MIN_FEATURE_COUNT)
    {
    	if (mDisplayFrames)
    	{
			cv::Point2f cog = calcCenterOfGravity(mFeatures[1]);
			float mse = calcMeanSquareError(mFeatures[1], cog);
			cv::circle(frame, mFaceCenter, 3, CV_RGB(255, 0, 0), CV_FILLED, CV_AA);
			cv::circle(frame, mFaceCenter, sqrtf(mse), CV_RGB(0, 120, 255), 3, CV_AA);

			for (i = 0; i < mFeatures[1].size(); i++)
				cv::circle(frame, mFeatures[1][i], 3, cv::Scalar(0,255,0), CV_FILLED, CV_AA);
    	}

#ifndef WEBCAM
    	sendHeadPosition(cloud);
#endif

        std::swap(mFeatures[1], mFeatures[0]);
    }
    else
    	mFeatures[0].clear();

    //if (mDisplayFrames)
    //	cv::imshow("FaceFocus", frame);
}

/**
 * Receives RGB images and displays them on screen.
 */
#ifdef WEBCAM
void FocusFace::imageCb(const sensor_msgs::ImagePtr &image_)
#else
void FocusFace::imageCb(const sensor_msgs::PointCloud2Ptr &cloud2)
#endif
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
#ifndef WEBCAM
	pcl::fromROSMsg(*cloud2, cloud);

    sensor_msgs::ImagePtr image_(new sensor_msgs::Image);
    pcl::toROSMsg(cloud, *image_);
#endif
    cv::Mat frame(image_->height, image_->width, CV_8UC3);
    imageToMat(image_, frame);

    cv::Mat tmp;
    if (mDisplayFrames)
    	frame.copyTo(tmp);

    if (mFeatures[0].size())
    {
    	trackFace(mPrevFrame, frame, cloud);
    	if (ros::Time::now().toSec() - mStartTime > MAX_TRACKING_TIME)
    		mFeatures[0].clear();
    }
    else
    {
    	detectFaces(frame, cloud);
    	if (mFeatures[0].size())
    		mStartTime = ros::Time::now().toSec();
    }

    if (mDisplayFrames)
    	tmp.copyTo(mPrevFrame);
    else
    	frame.copyTo(mPrevFrame);
}

bool FocusFace::setActiveCB(nero_msgs::SetActive::Request &req, nero_msgs::SetActive::Response &res)
{
	ROS_INFO("Active cb");
	if (mActive && req.active == false)
	{
		mImageSub.shutdown();
		ROS_INFO("Shutting down focus face.");
	}
	else if (mActive == false && req.active)
	{
		mImageSub = mNodeHandle.subscribe("/camera/depth_registered/points", 1, &FocusFace::imageCb, this);
		ROS_INFO("Starting up focus face.");
	}

	mActive = req.active;
	return true;
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "FocusFace");

	if (argc < 2)
	{
		ROS_ERROR("Invalid input arguments.");
		return 0;
	}

	FocusFace focusFace(argv[1], argc > 2 ? argv[2] : NULL, argc > 3 ? argv[3] : NULL);

	int sleep_rate;
	focusFace.getNodeHandle()->param<int>("node_sleep_rate", sleep_rate, 50);
	ros::Rate sleep(sleep_rate);

	while (ros::ok())
	{
		if (focusFace.isActive())
			sleep.sleep();
		ros::spinOnce();
	}
}
