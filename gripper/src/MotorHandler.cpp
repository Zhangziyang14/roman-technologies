#include <gripper/MotorHandler.h>

/**
 * Called when a motor command is received over the motor topic.
 */
void MotorHandler::gripperCB(const nero_msgs::GripperControl &mc)
{
    // some wrong message?
    if (mc.mode == CM_NONE)
        return;

    // Are we changing modes? Then let the motor know.
    if (mc.mode != cmode)
    {
        cmode = ControlMode(mc.mode);
        mGripperMotor.setMode(cmode);
    }
    // pass the value to the motor
    switch (mc.mode)
    {
    case CM_TORQUE_MODE:
    	mGripperMotor.setTorque(mc.value);
        break;

    case CM_CURRENT_MODE:
        mGripperMotor.setCurrent(mc.value);
        break;

    default:
        break;
    }

    // Do we need to sleep for a bit and then disable the motor ? (Use threads?)
    if (mc.waitTime)
    {
        usleep(mc.waitTime * 1000);
        mGripperMotor.setMode(CM_STOP_MODE);
        mGripperMotor.setMode(cmode);
    }
}

/**
 * Initalize MotorHandler and its attributes.
*/
void MotorHandler::init(char *path)
{
	//Initialise subscribers
	mMotorSub = mNodeHandle.subscribe("/gripperMotorTopic", 1, &MotorHandler::gripperCB, this);

	mGripperMotor.init(path);
	ROS_INFO("Initialising completed.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripperMotorHandler");

    char *path=NULL;
    if (argc == 2)
        path = argv[1];
 
    MotorHandler motorHandler;
    motorHandler.init(path);

    ros::spin();
    return 0;
}

