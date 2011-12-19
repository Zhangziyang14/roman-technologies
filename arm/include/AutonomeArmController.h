#ifndef AutonomeArmController_h
#define AutonomeArmController_h

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif


#include <ros/ros.h>



class AutonomeArmController
{
protected:
	ros::NodeHandle mNodeHandle;

public:
	AutonomeArmController(): mNodeHandle(""){}
	~AutonomeArmController()
	{
		mNodeHandle.shutdown();
	}

	void init();
};


#endif
