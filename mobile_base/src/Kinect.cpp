#include "Util.h"

/**
 * Scales the color of the disparity image so the closest point is red and the furthest point is blue.
 */
void colorizeDisparity( const cv::Mat& gray, cv::Mat& rgb, double maxDisp=-1.f, float S=1.f, float V=1.f )
{
	CV_Assert( !gray.empty() );
	CV_Assert( gray.type() == CV_8UC1 );

	if( maxDisp <= 0 )
	{
		maxDisp = 0;
		minMaxLoc( gray, 0, &maxDisp );
	}

	rgb.create( gray.size(), CV_8UC3 );
	rgb = cv::Scalar::all(0);
	if (maxDisp < 1)
		return;

	for (int y = 0; y < gray.rows; y++)
	{
		for (int x = 0; x < gray.cols; x++)
		{
			uchar d = gray.at<uchar>(y,x);
			unsigned int H = ((uchar)maxDisp - d) * 240 / (uchar)maxDisp;

			unsigned int hi = (H/60) % 6;
			float f = H/60.f - H/60;
			float p = V * (1 - S);
			float q = V * (1 - f * S);
			float t = V * (1 - (1 - f) * S);

			cv::Point3f res;

			if( hi == 0 ) //R = V,	G = t,	B = p
				res = cv::Point3f(p, t, V);
			if( hi == 1 ) // R = q,	G = V,	B = p
				res = cv::Point3f(p, V, q);
			if( hi == 2 ) // R = p,	G = V,	B = t
				res = cv::Point3f(t, V, p);
			if( hi == 3 ) // R = p,	G = q,	B = V
				res = cv::Point3f(V, q, p);
			if( hi == 4 ) // R = t,	G = p,	B = V
				res = cv::Point3f(V, p, t);
			if( hi == 5 ) // R = V,	G = p,	B = q
				res = cv::Point3f(q, p, V);

			uchar b = (uchar)(std::max(0.f, std::min(res.x, 1.f)) * 255.f);
			uchar g = (uchar)(std::max(0.f, std::min(res.y, 1.f)) * 255.f);
			uchar r = (uchar)(std::max(0.f, std::min(res.z, 1.f)) * 255.f);

			rgb.at<cv::Point3_<uchar> >(y,x) = cv::Point3_<uchar>(b, g, r);
		}
	}
}

/**
 * Returns the max distance of the kinect depth sensor.
 */
float getMaxDisparity(cv::VideoCapture& capture)
{
	const int minDistance = 400; // mm
	float b = (float)capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE ); // mm
	float F = (float)capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ); // pixels
	return b * F / minDistance;
}

/**
 * Constantly grabs images from the Kinect and performs operations on these images if necessary.
 */
void kinectLoop(cv::VideoCapture *capture)
{
	bool quit = false;
	DisplayType displayType = DISPLAY_TYPE_DEPTH;

	while (quit == false)
	{
		cv::Mat image, pointCloud;
		IplImage iplImage;

		if (!capture->grab())
		{
			std::cout << "Can not grab images." << std::endl;
			return;
		}

		switch (displayType)
		{
		case DISPLAY_TYPE_RGB:
			if (capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE))
			{
				iplImage = image;
				cvShowImage(WINDOW_NAME, &iplImage);
			}
			break;
		case DISPLAY_TYPE_GRAY:
			if (capture->retrieve(image, CV_CAP_OPENNI_GRAY_IMAGE))
			{
				iplImage = image;
				cvShowImage(WINDOW_NAME, &iplImage);
			}
			break;
		case DISPLAY_TYPE_DEPTH:
			if (capture->retrieve(image, CV_CAP_OPENNI_BGR_IMAGE) && capture->retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP))
			{
				IplImage rgb = image;
				IplImage pc = pointCloud;
				cvFlip(&rgb, &rgb, 1);
				cvFlip(&pc, &pc, 1);

				cv::Point p = cvPoint(rgb.width >> 1, rgb.height >> 1);
				cvDrawCircle(&rgb, p, 5, cvScalar(0, 255, 0), 1, CV_AA);

				int minDepth = 9999;
				cv::Point closestPoint;
				for (int y = 0; y < pc.height; y++)
				{
					for (int x = 0; x < pc.width; x++)
					{
						int depth = getDepthFromCloud(x, y, &pc);
						if (depth == 0)
							continue;

						if (*getPixelF32(x, y, &pc, 0) > -ROBOT_RADIUS &&
							*getPixelF32(x, y, &pc, 0) < ROBOT_RADIUS &&
							*getPixelF32(x, y, &pc, 1) < 0.f)
						{
							*getPixelU8(x, y, &rgb, 0) = 255;
							if (depth < minDepth)
							{
								minDepth = depth;
								closestPoint = cvPoint(x, y);
							}
						}
					}
				}

				//uint16 depth = getDepthFromCloud(p, &pc);
				//std::cout << "Depth at center: " << depth << std::endl;
				if (minDepth != 9999)
				{
					cvDrawCircle(&rgb, closestPoint, 5, cvScalar(0, 0, 255), 1, CV_AA);
					std::cout << "Closest obstacle at :" << minDepth << std::endl;
				}

				cvShowImage(WINDOW_NAME, &rgb);
			}
			break;
		case DISPLAY_TYPE_DISPARITY:
			if (capture->retrieve(image, CV_CAP_OPENNI_DISPARITY_MAP))
			{
				//if( isColorizeDisp )
				{
					cv::Mat colorDisparityMap;
					colorizeDisparity(image, colorDisparityMap, /*isFixedMaxDisp ? getMaxDisparity(capture) :*/ -1);
					cv::Mat validColorDisparityMap;
					colorDisparityMap.copyTo(validColorDisparityMap, image != 0);
					imshow(WINDOW_NAME, validColorDisparityMap);
				}
			}
			break;
		default:
			break;
		}

		int key = cv::waitKey(30);
		switch (key)
		{
		// Esc
		case 27:
			quit = true;
			break;
		// Space
		case 32:
			displayType = DisplayType((displayType + 1) % DISPLAY_TYPE_TOTAL);
			break;
		default:
			break;
		}
	}
}

int main(int argc, char* argv[])
{
	std::cout << "Kinect opening ..." << std::endl;
	cv::VideoCapture capture(CV_CAP_OPENNI);
	if( !capture.isOpened() )
	{
		std::cout << "Can not open a capture object." << std::endl;
		return -1;
	}
	std::cout << "done." << std::endl;

	capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ); // default

	// Print some avalible Kinect settings.
	std::cout << "\nDepth generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FRAME_MAX_DEPTH\t" << capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm" << std::endl <<
			"FPS\t" << capture.get(CV_CAP_PROP_FPS) << std::endl;

	std::cout << "\nImage generator output mode:" << std::endl <<
			"FRAME_WIDTH\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH) << std::endl <<
			"FRAME_HEIGHT\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT) << std::endl <<
			"FPS\t" << capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS) << std::endl;

	cvNamedWindow(WINDOW_NAME);
	kinectLoop(&capture);

	return 0;
}
