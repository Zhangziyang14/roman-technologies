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
		mImageTransport(mNodeHandle), mStartTime(0)
{
	image_transport::TransportHints hints("compressed", ros::TransportHints(), mNodeHandle);
	mImageSub 			= mImageTransport.subscribe("/camera/rgb/image_color", 5, &FocusFace::imageCb, this, hints);
	mHeadPosSub 		= mNodeHandle.subscribe("/headPositionFeedbackTopic", 1, &FocusFace::headPositionCb, this);
	mHeadSpeedSub 		= mNodeHandle.subscribe("/headSpeedFeedbackTopic", 1, &FocusFace::headSpeedCb, this);
	mHeadPosPub 		= mNodeHandle.advertise<nero_msgs::PitchYaw>("/cmd_head_position", 1);

	mActiveServer 		= mNodeHandle.advertiseService("/set_focus_face", &FocusFace::setActiveCB, this);
	mImageControlClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/KinectServer/RGBControl");
	mCloudSaveClient	= mNodeHandle.serviceClient<nero_msgs::SetActive>("/KinectServer/CloudSaveControl");
	mQueryCloudClient	= mNodeHandle.serviceClient<nero_msgs::QueryCloud>("/KinectServer/QueryCloud");

	mFaceCenter.x = mFaceCenter.y = 0.f;
	mFaceCenterPct.x = mFaceCenterPct.y = 0.f;
	mCurrentOrientation.pitch = 0.f;
	mCurrentOrientation.yaw = 0.f;
	mActive = false;
	setRGBOutput(false);
	setCloudSave(false);

    if (mFrontalFaceCascade.load(frontal_face)	 == false)
        ROS_ERROR("Could not load frontal face classifier cascade");
    if (profile_face == NULL || mProfileCascade.load(profile_face) == false)
    	ROS_WARN("Could not load profile face classifier cascade");
    if (profile_face == NULL || mFrontalFace2Cascade.load(frontal_face_2) == false)
    	ROS_WARN("Could not load frontal face2 classifier cascade");

    mNodeHandle.param<double>("/FocusFace/image_scale", mScale, 1.0);
    mNodeHandle.param<bool>("/FocusFace/display_frames", mDisplayFrames, true);
    mNodeHandle.param<bool>("/FocusFace/detect_only", mDetectOnly, false);
    mNodeHandle.param<bool>("/FocusFace/send_head_position", mSendHeadPosition, true);

    if (mDisplayFrames)
    {
		cv::startWindowThread();
		cvNamedWindow("FaceFocus", 1);
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

void FocusFace::sendHeadPosition()
{
	if (canMoveHead() == false)
		return;

    //pcl::PointXYZRGB p = cloud.at(mFaceCenter.x, mFaceCenter.y);
	nero_msgs::QueryCloud query;
	geometry_msgs::Point index;
	index.x = mFaceCenterPct.x;
	index.y = mFaceCenterPct.y;
	query.request.indices.push_back(index);

	if (mQueryCloudClient.call(query) == false)
	{
		ROS_WARN("Failed to call cloud save server.");
		return;
	}

	if (query.response.points.size() == 0)
	{
		ROS_WARN("QueryCloud returned 0 points.");
		return;
	}

	geometry_msgs::Point p = query.response.points[0];
    if (p.z == 0.f)
    {
    	ROS_WARN("Invalid face center.");
    	return;
    }

    double pitch = -atan(p.y / p.z);
    double yaw = -atan(p.x / p.z);

    nero_msgs::PitchYaw msg;
    msg.pitch = pitch + mCurrentOrientation.pitch;
    msg.yaw = yaw + mCurrentOrientation.yaw;
    if (isnan(msg.pitch) || isnan(msg.yaw))
    	return;

    mHeadPosPub.publish(msg);
}

void FocusFace::detectFaces(cv::Mat &frame)
{
	uint16_t minDepth = -1;
	size_t minIndex = -1;
	cv::Mat gray_frame, scaled_frame;
	cv::cvtColor(frame, gray_frame, CV_BGR2GRAY);
	cv::equalizeHist(gray_frame, gray_frame);
	cv::resize(gray_frame, scaled_frame, cv::Size(gray_frame.cols / mScale, gray_frame.rows / mScale), 0, 0, cv::INTER_LINEAR);

	std::vector<cv::Rect> faces;
	mFrontalFaceCascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(30, 30));

	if (faces.size() == 0 && mProfileCascade.empty() == false)
		mProfileCascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(30, 30));
	if (faces.size() == 0 && mFrontalFace2Cascade.empty() == false)
		mFrontalFace2Cascade.detectMultiScale(scaled_frame, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(30, 30));

    for (size_t i = 0; i < faces.size(); i++)
    {
    	faces[i].x *= mScale;
    	faces[i].y *= mScale;
    	faces[i].height *= mScale;
    	faces[i].width *= mScale;

    	cv::rectangle(frame, faces[i], CV_RGB(255, 0, 0), 3);

//#ifdef WEBCAM
		minDepth = 0;
		minIndex = i;
		break;
//#endif

    	// look for the closest face
    	uint16_t depth = 0;//getDepthFromCloud(faces[i].x + (faces[i].width >> 1), faces[i].y + (faces[i].height >> 1), &cloud);
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
    	mFaceCenterPct.x = mFaceCenter.x / frame.cols;
    	mFaceCenterPct.y = mFaceCenter.y / frame.rows;
    	cv::Mat mask = cv::Mat::zeros(cv::Size(gray_frame.cols, gray_frame.rows), CV_8UC1);

    	cv::circle(mask, mFaceCenter, radius, cv::Scalar(255), CV_FILLED);

    	if (mDetectOnly == false)
    	{
			mFeatures[0].clear();
			cv::goodFeaturesToTrack(gray_frame, mFeatures[0], MAX_CORNERS, 0.01, 8, mask);

			if (mFeatures[0].size() > MIN_FEATURE_COUNT)
			{
				if (mDisplayFrames)
				{
					cv::Point2f cog = calcCenterOfGravity(mFeatures[0]);
					float mse = calcMeanSquareError(mFeatures[0], cog);
					cv::circle(frame, mFaceCenter, sqrtf(mse), CV_RGB(0, 120, 255), 3, CV_AA);

					for (size_t j = 0; j < mFeatures[0].size(); j++)
						cv::circle(frame, mFeatures[0][j], 3, CV_RGB(0, 125, 125), 1, CV_AA);
				}
			}
			else
				mFeatures[0].clear();
    	}

    	if (mSendHeadPosition)
    		sendHeadPosition();
    }

    if (mDisplayFrames)
    	cv::imshow("FaceFocus", frame);

}

void FocusFace::trackFace(cv::Mat &prevFrame, cv::Mat &frame)
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
	mFaceCenterPct.x = mFaceCenter.x / frame.cols;
	mFaceCenterPct.y = mFaceCenter.y / frame.rows;

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

    	if (mSendHeadPosition)
    		sendHeadPosition();

        std::swap(mFeatures[1], mFeatures[0]);
    }
    else
    	mFeatures[0].clear();

    if (mDisplayFrames)
    	cv::imshow("FaceFocus", frame);
}

/**
 * Receives RGB images and displays them on screen.
 */
void FocusFace::imageCb(const sensor_msgs::ImageConstPtr &image_)
{
    cv::Mat frame(image_->height, image_->width, CV_8UC3);
    imageToMat(image_, frame);

    cv::Mat tmp;
    if (mDisplayFrames)
    	frame.copyTo(tmp);

    if (mDetectOnly == false && mFeatures[0].size())
    {
    	trackFace(mPrevFrame, frame);
    	if (ros::Time::now().toSec() - mStartTime > MAX_TRACKING_TIME)
    		mFeatures[0].clear();
    }
    else
    {
    	detectFaces(frame);
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
		setRGBOutput(false);
		setCloudSave(false);
		ROS_INFO("Shutting down focus face.");
	}
	else if (mActive == false && req.active)
	{
		setRGBOutput(true);
		setCloudSave(true);
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
		sleep.sleep();
		ros::spinOnce();
	}

	focusFace.setRGBOutput(false);
	focusFace.setCloudSave(false);
}
