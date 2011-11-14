/*
 * PathTester.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "PathFinder.h"
#include <gtest/gtest.h>

ros::Publisher odomTest_pub;
ros::Publisher twistTest_pub;
ros::Publisher targetTest_pub;
ros::Subscriber pathTest_sub;

logical_unit::target target_msg;
nav_msgs::Odometry current_msg;

double resultAngle, resultDistance;


void resultCBTest(const logical_unit::path& msg)
{
	resultAngle 	= msg.angle;
	resultDistance 	= msg.distance;
}

///Test conversion when robot has not moved yet, with target left of robot
TEST(targetLeftOfRobot, originTest)
{
	//robot is still in origin of original coordinate system
	current_msg.pose.pose.position.x = 0;
	current_msg.pose.pose.position.y = 0;

	target_msg.x = -4;
	target_msg.y = 0;

	odomTest_pub.publish(current_msg);
	targetTest_pub.publish(target_msg);

	ros::Rate rate(10);

	unsigned int cycles = 10;
	while(cycles-- && !called)
	{
		ros::spinOnce();
		rate.sleep();
	}

	EXPECT_TRUE(resultDistance == 4 && resultAngle == -90);
}

//TODO Tests in origin with targets right, above and under robot

//TODO Tests in upper RHP with targets right, above and under robot
//TODO Tests in upper LHP with targets right, above and under robot
//TODO Tests in lower RHP with targets right, above and under robot
//TODO Tests in lower LHP with targets right, above and under robot

//TODO Tests in origin with positive rotated coordinate axes
//TODO Tests in RHP with positive rotated coordinate axes
//TODO Tests in LHP with positive rotated coordinate axes

//TODO Tests in origin with negative rotated coordinate axes
//TODO Tests in RHP with negative rotated coordinate axes
//TODO Tests in LHP with negative rotated coordinate axes


//Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	ros::init(argc, argv, "pathTest");

	ros::NodeHandle node;
	odomTest_pub    = node.advertise<roman::Key>("odom", 10);
	targetTest_pub  = node.advertise<logical_unit::target>("pathTopic", 10);
	twistTest_pub   = node.advertise<geometry_msgs::Twist>("speedFeedbackTopic", 10);

	pathTest_sub  = node.subscribe("processedPathTopic", 10, resultCBTest);

	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
