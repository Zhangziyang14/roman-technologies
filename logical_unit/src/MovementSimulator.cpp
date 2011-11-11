#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

ros::Publisher *odom_publisher;
tf::TransformBroadcaster *odom_broadcaster;
ros::Time current_time, last_time;

// globally defined position and rotation
double th = 0.0;
double x = 0.0;
double y = 0.0;

void twistCb(const geometry_msgs::Twist &twist)
{
	// calculate velocities
	double vx = twist.linear.x * cos(th);
	double vy = twist.linear.x * sin(th);
	double vth = twist.angular.z;

	//ROS_INFO("vx: %lf, vy: %lf, vth: %lf, lin.x: %lf, ang.z: %lf, th: %lf", vx, vy, vth, twist.linear.x, twist.angular.z, th);

	current_time = ros::Time::now();

	//compute odometry given the velocities of the robot
	double dt = (current_time - last_time).toSec();
	double delta_x = vx * dt;
	double delta_y = vy * dt;
	double delta_th = vth * dt;

	//ROS_INFO("dx: %lf, dy: %lf, dth: %lf, dt: %lf", delta_x, delta_y, delta_th, dt);

	// adjust position and orientation
	x += delta_x;
	y += delta_y;
	th += delta_th;

	//ROS_INFO("x: %lf, y: %lf, th: %lf", x, y, th);

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	//ROS_INFO("x: %f, y: %f, th: %f", x, y, th);

	//send the transform
	odom_broadcaster->sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	//This populates the message with odometry data and sends it out over the wire. We'll set the child_frame_id of the message to be the "base_link" frame since that's the coordinate frame we're sending our velocity information in.

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_publisher->publish(odom);

	last_time = current_time;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "MovementSimulator");
	ros::NodeHandle node;
	odom_publisher = new ros::Publisher(node.advertise<nav_msgs::Odometry>("odom", 50));
	odom_broadcaster = new tf::TransformBroadcaster();
	ros::Subscriber twist_sub = node.subscribe("/speedFeedbackTopic", 1, twistCb);
	
	// initialize timer
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	//ros::Rate rate(100.0);
	while (node.ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

