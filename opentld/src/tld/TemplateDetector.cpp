#include "TemplateDetector.h"

// based on http://opencv.willowgarage.com/wiki/FastMatchTemplate

namespace tld {

TemplateDetector::TemplateDetector() :
	mDetectionResult(NULL),
	mInitialised(false),
	mScale(0.5f),
	mConfThresh(85),
	mEnabled(true),
	mShowOutput(false)
{
}

void TemplateDetector::setShowOutput(bool show)
{
	if (mShowOutput && show == false)
	{
		// destroy output
		cv::destroyWindow("TemplateResult");
		cv::destroyWindow("Template");
	}
	else if (mShowOutput == false && show)
	{
		// show output
		cv::namedWindow("TemplateResult", 1);
		cv::namedWindow("Template", 1);
	}

	mShowOutput = show;
}

void TemplateDetector::findMultipleMaxLoc(const cv::Mat& image, cv::Point** locations, int numMaxima)
{
    // initialize input variable locations
    *locations = new cv::Point[numMaxima];

    // create array for tracking maxima
    float* maxima = new float[numMaxima];
    for(int i = 0; i < numMaxima; i++)
    {
        maxima[i] = 0.0;
    }

    cv::Size size = image.size();

    // extract the raw data for analysis
    for(int y = 0; y < size.height; y++)
    {
        for(int x = 0; x < size.width; x++)
        {
            float data = image.at<float>(y, x);

            // insert the data value into the array if it is greater than any of the
            //  other array values, and bump the other values below it, down
            for(int j = 0; j < numMaxima; j++)
            {
                // require at least 50% confidence on the sub-sampled image
                // in order to make this as fast as possible
                if(data > 0.5 && data > maxima[j])
                {
                    // move the maxima down
                    for(int k = numMaxima - 1; k > j; k--)
                    {
                        maxima[k] = maxima[k-1];
                        (*locations)[k] = ( *locations )[k-1];
                    }

                    // insert the value
                    maxima[j] = data;
                    (*locations)[j].x = x;
                    (*locations)[j].y = y;
                    break;
                }
            }
        }
    }

    delete [] maxima;
}

bool TemplateDetector::fastMatchTemplate(
		const cv::Mat			&source,
        const cv::Mat			&target,
        std::vector<cv::Point>	*foundPointsList,
        std::vector<double>		*confidencesList,
        int						matchPercentage,
        bool					findMultipleTargets,
        int						numMaxima,
        int						numDownPyrs,
        int						searchExpansion)
{
    // make sure that the template image is smaller than the source
    if (target.size().width > source.size().width ||
        target.size().height > source.size().height)
    {
        std::cout << "Source image must be larger than target image." << std::endl;
        return false;
    }

    if (source.depth() != target.depth())
    {
    	std::cout << "Source image and target image must have same depth." << std::endl;
        return false;
    }

    if (source.channels() != target.channels())
    {
    	std::cout << "Source image and target image must have same number of channels." << std::endl;
        return false;
    }

    cv::Size sourceSize = source.size();
    cv::Size targetSize = target.size();

    // create copies of the images to modify
    cv::Mat copyOfSource = source.clone();
    cv::Mat copyOfTarget = target.clone();

    // down pyramid the images
    for (int ii = 0; ii < numDownPyrs; ii++)
    {
        // start with the source image
        sourceSize.width  = (sourceSize.width  + 1) / 2;
        sourceSize.height = (sourceSize.height + 1) / 2;

        cv::Mat smallSource(sourceSize, source.type());
        cv::pyrDown(copyOfSource, smallSource);

        // prepare for next loop, if any
        copyOfSource = smallSource.clone();

        // next, do the target
        targetSize.width  = (targetSize.width  + 1) / 2;
        targetSize.height = (targetSize.height + 1) / 2;

        cv::Mat smallTarget(targetSize, target.type());
        cv::pyrDown(copyOfTarget, smallTarget);

        // prepare for next loop, if any
        copyOfTarget = smallTarget.clone();
    }

    // perform the match on the shrunken images
    cv::Size smallTargetSize = copyOfTarget.size();
    cv::Size smallSourceSize = copyOfSource.size();

    cv::Size resultSize;
    resultSize.width = smallSourceSize.width - smallTargetSize.width + 1;
    resultSize.height = smallSourceSize.height - smallTargetSize.height + 1;

    cv::Mat result(resultSize, CV_32FC1);
    cv::matchTemplate(copyOfSource, copyOfTarget, result, CV_TM_CCOEFF_NORMED);

    // find the top match locations
    cv::Point *locations = NULL;
    findMultipleMaxLoc(result, &locations, numMaxima);

    // search the large images at the returned locations
    sourceSize = source.size();
    targetSize = target.size();

    // create a copy of the source in order to adjust its ROI for searching
    for (int currMax = 0; currMax < numMaxima; currMax++)
    {
        // transform the point to its corresponding point in the larger image
        locations[currMax].x *= (int)pow(2.0f, numDownPyrs);
        locations[currMax].y *= (int)pow(2.0f, numDownPyrs);
        locations[currMax].x += targetSize.width / 2;
        locations[currMax].y += targetSize.height / 2;

        const cv::Point &searchPoint = locations[currMax];

        // if we are searching for multiple targets and we have found a target or
        //  multiple targets, we don't want to search in the same location(s) again
        if (findMultipleTargets && !foundPointsList->empty())
        {
            bool thisTargetFound = false;

            int numPoints = foundPointsList->size();
            for (int currPoint = 0; currPoint < numPoints; currPoint++)
            {
                const cv::Point &foundPoint = (*foundPointsList)[currPoint];
                if (abs(searchPoint.x - foundPoint.x) <= searchExpansion * 2 &&
                    abs(searchPoint.y - foundPoint.y) <= searchExpansion * 2)
                {
                    thisTargetFound = true;
                    break;
                }
            }

            // if the current target has been found, continue onto the next point
            if (thisTargetFound)
                continue;
        }

        // set the source image's ROI to slightly larger than the target image,
        //  centred at the current point
        cv::Rect searchRoi;
        searchRoi.x = searchPoint.x - (target.size().width) / 2 - searchExpansion;
        searchRoi.y = searchPoint.y - (target.size().height) / 2 - searchExpansion;
        searchRoi.width = target.size().width + searchExpansion * 2;
        searchRoi.height = target.size().height + searchExpansion * 2;

        // make sure ROI doesn't extend outside of image
        searchRoi.x = std::max(0, searchRoi.x);
        searchRoi.y = std::max(0, searchRoi.y);
        searchRoi.width = std::max(1, std::min(sourceSize.width - searchRoi.x - 1, searchRoi.width));
        searchRoi.height = std::max(1, std::min(sourceSize.height - searchRoi.y - 1, searchRoi.height));

        cv::Mat searchImage = cv::Mat(source, searchRoi);

        // perform the search on the large images
        resultSize.width = searchRoi.width - target.size().width + 1;
        resultSize.height = searchRoi.height - target.size().height + 1;

        result = cv::Mat(resultSize, CV_32FC1);
        cv::matchTemplate(searchImage, target, result, CV_TM_CCOEFF_NORMED);

        // find the best match location
        double minValue, maxValue;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minValue, &maxValue, &minLoc, &maxLoc);
        maxValue *= 100;

        // transform point back to original image
        maxLoc.x += searchRoi.x + target.size().width / 2;
        maxLoc.y += searchRoi.y + target.size().height / 2;

        if (maxValue >= matchPercentage)
        {
            // add the point to the list
            foundPointsList->push_back(maxLoc);
            confidencesList->push_back(maxValue);

            // if we are only looking for a single target, we have found it, so we
            //  can return
            if (!findMultipleTargets)
                break;
        }
    }

    //if (foundPointsList->empty())
    //	std::cout << "Target was not found to required confidence of " << matchPercentage << std::endl;

    delete [] locations;

    return true;
}

void TemplateDetector::drawFoundTargets(
		cv::Mat*             			image,
        const cv::Size&           		size,
        const std::vector<cv::Point>	&pointsList,
        const std::vector<double>		&confidencesList,
        cv::Scalar						color)
{
    int numPoints = pointsList.size();
    for (int currPoint = 0; currPoint < numPoints; currPoint++)
    {
        const cv::Point &point = pointsList[currPoint];

        // write the confidences to stdout
        //printf("Target found at (%d, %d), with confidence = %3.3f %%.\n", point.x, point.y, confidencesList[currPoint]);

        // draw a circle at the center
        cv::circle(*image, point, 2, color);

        // draw a rectangle around the found target
        cv::Point topLeft;
        topLeft.x = point.x - size.width / 2;
        topLeft.y = point.y - size.height / 2;

        cv::Point bottomRight;
        bottomRight.x = point.x + size.width / 2;
        bottomRight.y = point.y + size.height / 2;

        cv::rectangle(*image, topLeft, bottomRight, color);
    }
}

void TemplateDetector::init(cv::Mat frame, cv::Rect r)
{
	if (mEnabled == false)
		return;

	if (frame.empty())
	{
		std::cout << "Frame in TemplateDetector is empty!" << std::endl;
		return;
	}

	cv::Mat tmp;
	frame(r).copyTo(tmp);
	cv::resize(tmp, mTemplate, cv::Size(tmp.cols * mScale, tmp.rows * mScale), 0, 0, cv::INTER_LINEAR);

	if (mShowOutput)
		cv::imshow("Template", mTemplate);
	mInitialised = true;
}

void TemplateDetector::release()
{
	mInitialised = false;
}

bool TemplateDetector::detect(cv::Mat frame)
{
	if (mInitialised == false || mEnabled == false)
		return false;

	cv::Mat scaledFrame;
	cv::resize(frame, scaledFrame, cv::Size(frame.cols * mScale, frame.rows * mScale), 0, 0, cv::INTER_LINEAR);
	cv::Mat result(mTemplate.cols - mTemplate.cols + 1, mTemplate.rows - mTemplate.rows + 1, CV_32FC1);

	/// Do the Matching and Normalize
	//cv::matchTemplate(scaledFrame, mTemplate, result, CV_TM_CCOEFF_NORMED);
    std::vector<cv::Point> foundPointsList;
    std::vector<double> confidencesList;
	if (fastMatchTemplate(scaledFrame, mTemplate, &foundPointsList, &confidencesList) == false)
	{
		std::cout << "Fast template matching failed." << std::endl;
		return false;
	}

	if (mShowOutput)
	{
		drawFoundTargets(&scaledFrame, mTemplate.size(), foundPointsList, confidencesList);
		cv::imshow("TemplateResult", scaledFrame);
		cv::imshow("Template", mTemplate);
	}

	if (foundPointsList.size())
	{
		cv::Point p = foundPointsList[0];
		cv::Rect *r = new cv::Rect((p.x - mTemplate.cols / 2) / mScale, (p.y - mTemplate.rows / 2) / mScale,
				mTemplate.cols / mScale, mTemplate.rows / mScale);

        r->x = std::max(0, r->x);
        r->y = std::max(0, r->y);
        r->width = std::max(1, std::min(frame.cols - r->x - 1, r->width));
        r->height = std::max(1, std::min(frame.rows - r->y - 1, r->height));

        mDetectionResult->detectorBB = r;
		mDetectionResult->numClusters = 1;
		return true;
	}

	return false;
}

}  /* namespace tld */

