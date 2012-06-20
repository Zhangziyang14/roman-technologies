/*
 * DetectorTemplate.h
 *
 *  Created on: Jun 05, 2012
 *      Author: hgaiser
 */

#ifndef TEMPLATEDETECTOR_H_
#define TEMPLATEDETECTOR_H_

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "DetectionResult.h"

namespace tld {

class TemplateDetector
{
public:
	TemplateDetector();

	void init(cv::Mat frame, cv::Rect r);
	void release();

	bool detect(cv::Mat frame);
	void setShowOutput(bool show);

	DetectionResult* mDetectionResult;

	inline bool isInitialised() { return mInitialised; };

private:
	cv::Mat mTemplate;
	bool mInitialised;
	float mScale;
	int mConfThresh;

public:
	bool mEnabled;
	bool mShowOutput;

private:
	bool fastMatchTemplate(const cv::Mat &source, const cv::Mat &target, std::vector<cv::Point> *foundPointsList,
			std::vector<double> *confidencesList, int matchPercentage = 70, bool findMultipleTargets = false,
			int numMaxima = 10, int numDownPyrs = 2, int searchExpansion = 15);
	void findMultipleMaxLoc(const cv::Mat &image, cv::Point** locations, int numMaxima);
	void drawFoundTargets(cv::Mat *image, const cv::Size &size, const std::vector<cv::Point> &pointsList,
			const std::vector<double> &confidencesList, cv::Scalar color = CV_RGB(0, 255, 0));
};

} /* namespace tld */
#endif /* TEMPLATEDETECTOR_H_ */
