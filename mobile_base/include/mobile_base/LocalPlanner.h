/*
 * LocalPlanner.h
 *
 *  Created on: Dec 12, 2011
 *      Author: hans
 */

#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

#include "ros/ros.h"

class LocalPlanner: public BaseLocalPlanner
{
private:
	ros::NodeHandle mNodeHandle;

public:
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) { return false; };
    virtual bool isGoalReached() { return true; };
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
};

#endif /* LOCALPLANNER_H_ */
