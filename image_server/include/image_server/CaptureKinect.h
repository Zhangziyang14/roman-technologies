/*
 * CaptureKinect.h
 *
 *  Created on: May 6, 2012
 *      Author: hans
 */

#ifndef CAPTUREKINECT_H_
#define CAPTUREKINECT_H_

#include "ros/ros.h"

#include "XnCppWrapper.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

class CaptureKinect
{
protected:
	xn::Context mContext;
    bool mIsContextOpened;
    bool mRGBGenerating;
    bool mDepthGenerating;
    const char *mConfigPath;

    XnUInt64 mShadowValue;
    XnUInt64 mNoSampleValue;

    xn::ProductionNode mProductionNode;
    xn::ScriptNode mScriptNode;

    // Data generators with its metadata
    xn::DepthGenerator mDepthGenerator;
    xn::DepthMetaData  mDepthMetaData;

    xn::ImageGenerator mImageGenerator;
    xn::ImageMetaData  mImageMetaData;
public:
	CaptureKinect(const char *filePath);
	~CaptureKinect();

	bool open();
	bool close();
	bool startDepth();
	bool stopDepth();
	bool startRGB();
	bool stopRGB();

	bool queryFrame(bool image, bool depth);
	cv::Mat getImage();
	cv::Mat getCloud(cv::Mat depth);
	cv::Mat getDepth();

	inline bool isOpened() { return mIsContextOpened; };
	inline bool isDepthGenerating() { return mDepthGenerating; };
	inline bool isRGBGenerating() { return mRGBGenerating; };

	inline xn::DepthGenerator* getDepthGenerator() { return &mDepthGenerator; };
};


#endif /* CAPTUREKINECT_H_ */
