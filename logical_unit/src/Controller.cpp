/*
 * Controller.cpp
 *
 *  Created on: 2011-11-13
 *      Author: wilson
 */

#include "Controller.h"

void Controller::init()
{
	//initialise subscribers

	//initialise publishers

}

int main(int argc, char **argv)
{
	// init ros and pathfinder
	ros::init(argc, argv, "controller");
	Controller controller;

	char *path = NULL;
	if (argc == 2)
		path = argv[1];

	controller.init();
	ros::spin();
}
