/*
 * CaptureKinect.cpp
 *
 *  Created on: May 6, 2012
 *      Author: hans
 */

#include "image_server/CaptureKinect.h"

CaptureKinect::CaptureKinect(const char *filePath) :
	mIsContextOpened(false),
	mConfigPath(filePath)
{
}

CaptureKinect::~CaptureKinect()
{
	if (isOpened() == false)
		return;

	mImageGenerator.Release();
	mDepthGenerator.Release();
	mContext.Release();
}

bool CaptureKinect::queryFrame(bool image, bool cloud)
{
	XnStatus rc = XN_STATUS_OK;

	// Read a new frame
	rc = mContext.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return false;
	}

	if (image)
		mImageGenerator.GetMetaData(mImageMetaData);

	if (cloud)
		mDepthGenerator.GetMetaData(mDepthMetaData);

	return true;
}

cv::Mat CaptureKinect::getImage()
{
    if (mImageMetaData.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
    {
        ROS_ERROR("Unsupported format of grabbed image.");
        return cv::Mat::zeros(0, 0, CV_8UC3);
    }

    cv::Mat rgbImage(mImageMetaData.YRes(), mImageMetaData.XRes(), CV_8UC3);
    const XnRGB24Pixel* pRgbImage = mImageMetaData.RGB24Data();

    memcpy(rgbImage.data, pRgbImage, rgbImage.total()*sizeof(XnRGB24Pixel));
    cv::cvtColor(rgbImage, rgbImage, CV_RGB2BGR);

    return rgbImage;
}

cv::Mat CaptureKinect::getCloud()
{
	cv::Mat depth(mDepthMetaData.YRes(), mDepthMetaData.XRes(), CV_16UC3);

    const XnDepthPixel* pDepthMap = mDepthMetaData.Data();
    memcpy(depth.data, pDepthMap, depth.cols*depth.rows*sizeof(XnDepthPixel));

    int cols = mDepthMetaData.XRes();
    int rows = mDepthMetaData.YRes();
    cv::Mat pointCloud_XYZ( rows, cols, CV_32FC3);

    cv::Ptr<XnPoint3D> proj = new XnPoint3D[cols*rows];
    cv::Ptr<XnPoint3D> real = new XnPoint3D[cols*rows];
    for( int y = 0; y < rows; y++ )
    {
        for( int x = 0; x < cols; x++ )
        {
            int ind = y*cols+x;
            proj[ind].X = (float)x;
            proj[ind].Y = (float)y;
            proj[ind].Z = depth.at<unsigned short>(y, x);
        }
    }
    mDepthGenerator.ConvertProjectiveToRealWorld(cols*rows, proj, real);

    cv::Point3f badPoint(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < cols; x++)
        {
            // Check for invalid measurements
            if (depth.at<unsigned short>(y, x) == 0) // not valid
                pointCloud_XYZ.at<cv::Point3f>(y,x) = badPoint;
            else
            {
                int ind = y*cols+x;
                pointCloud_XYZ.at<cv::Point3f>(y,x) = cv::Point3f(real[ind].X*0.001f, real[ind].Y*0.001f, real[ind].Z*0.001f); // from mm to meters
            }
        }
    }

	return pointCloud_XYZ;
}

bool CaptureKinect::open()
{
	if (isOpened())
		return false;

	XnStatus rc;
	xn::EnumerationErrors errors;
	rc = mContext.InitFromXmlFile(mConfigPath, mScriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		ROS_WARN("%s\n", strError);
		return false;
	}
	else if (rc != XN_STATUS_OK)
	{
		ROS_WARN("Open failed: %s\n", xnGetStatusString(rc));
		return false;
	}

	rc = mContext.FindExistingNode(XN_NODE_TYPE_DEPTH, mDepthGenerator);
	if (rc != XN_STATUS_OK)
	{
		ROS_WARN("No depth node exists! Check your XML.");
		return false;
	}

	rc = mContext.FindExistingNode(XN_NODE_TYPE_IMAGE, mImageGenerator);
	if (rc != XN_STATUS_OK)
	{
		ROS_WARN("No image node exists! Check your XML.");
		return false;
	}

	mDepthGenerator.GetMetaData(mDepthMetaData);
	mImageGenerator.GetMetaData(mImageMetaData);

	// Hybrid mode isn't supported in this sample
	if (mImageMetaData.FullXRes() != mDepthMetaData.FullXRes() || mImageMetaData.FullYRes() != mDepthMetaData.FullYRes())
	{
		ROS_WARN("The device depth and image resolution must be equal!\n");
		return false;
	}

	// RGB is the only image format supported.
	if (mImageMetaData.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		ROS_WARN("The device image format must be RGB24\n");
		return false;
	}

    if (mDepthGenerator.GetIntProperty( "ShadowValue", mShadowValue ) != XN_STATUS_OK)
    {
        ROS_WARN("CvCapture_OpenNI::readCamerasParams : Could not read property \"ShadowValue\"!");
        return false;
    }

    if (mDepthGenerator.GetIntProperty("NoSampleValue", mNoSampleValue ) != XN_STATUS_OK)
    {
        ROS_WARN("CvCapture_OpenNI::readCamerasParams : Could not read property \"NoSampleValue\"!");
        return false;
    }

	mContext.StopGeneratingAll();

	mIsContextOpened = true;
	mIsGenerating = false;

	return true;
}

bool CaptureKinect::close()
{
	if (isOpened() == false)
		return false;

	mDepthGenerator.Release();
	mImageGenerator.Release();
	mContext.Release();

	mIsContextOpened = false;
	mIsGenerating = false;
	return true;
}

bool CaptureKinect::start()
{
	if (isGenerating())
		return false;

	mContext.StartGeneratingAll();
	mIsGenerating = true;
	return true;
}

bool CaptureKinect::stop()
{
	if (isGenerating() == false)
		return false;

	mContext.StopGeneratingAll();
	mIsGenerating = false;
	return true;
}
