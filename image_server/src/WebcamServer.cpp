#include "image_server/OpenCVTools.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CompressedImage.h"

int main(int argc, char *argv[])
{
	// init ros
	ros::init(argc, argv, "WebcamServer");

	ros::NodeHandle nh;
	//image_transport::ImageTransport it(nh);

	//image_transport::Publisher pub = it.advertise("/camera/rgb/image_color", 1);
	ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>("/camera/rgb/image_color/compressed", 1);

	bool show_fps;
	double desired_fps;
	nh.param<bool>("/WebcamServer/show_fps", show_fps, false);
	nh.param<double>("/WebcamServer/fps", desired_fps, 30.0);
	double fps = 0.0;
	int fpsCount = 0;

	ros::Rate rate(desired_fps);

	cv::VideoCapture cap(CV_CAP_ANY);
	if (cap.isOpened() == false)
	{
		ROS_ERROR("Failed to open webcam.");
		return 0;
	}

	//int frameCount = 0;
	while (ros::ok())
	{
		rate.sleep();

		if (show_fps && cap.isOpened())
		{
			double cycleTime = rate.cycleTime().toSec();
			if (cycleTime != 0.0)
			{
				fps += (1.0 / cycleTime);
				fpsCount++;

				if (fpsCount == 10)
				{
					ROS_INFO("Rate: %lf", std::min(desired_fps, fps / fpsCount));
					fps = 0.0;
					fpsCount = 0;
				}
			}
		}

		if (pub.getNumSubscribers())
		{
			cv::Mat frame;
			cap >> frame;
			//pub.publish(OpenCVTools::matToImage(frame));

			sensor_msgs::CompressedImage rgb_c;
			rgb_c.header.stamp = ros::Time::now();
			std::vector<int> p;
			p.push_back(CV_IMWRITE_JPEG_QUALITY);
			p.push_back(90);
			cv::imencode(".jpg", frame, rgb_c.data, p);

			pub.publish(rgb_c);

		    /*char path[255];
			sprintf(path, "/home/hans/tracking/images/frame%.4d.jpg", frameCount);
			frameCount++;

			cv::imwrite(path, frame);*/
		}

		ros::spinOnce();
	}
}
