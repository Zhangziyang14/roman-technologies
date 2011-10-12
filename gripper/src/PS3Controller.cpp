/*
 * PS3Controller.cpp
 *
 *  Created on: Oct 11, 2011
 *      Author: hans
 */

#include "PS3Controller.h"

void PS3Controller::init()
{
	Controller::init();

	// subscriber to ps3 key events
	mKey_sub    = mNodeHandle.subscribe("joy", 10, &PS3Controller::keyCB, this);
}

/**
 * Called when sensor should be activated/deactivated.
*/
void PS3Controller::keyCB(const sensor_msgs::Joy& msg)
{
    //ROS_INFO("Key Received. Id = %d, value = %d", msg->keys[i], msg->values[i]);

    gripper::MotorControl mc;
    std_msgs::Empty empty_msg;

    //No button is pressed, so sum of vector is zero
    if(std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0)
    {
        // if the key was one of the shoulder buttons, release the motor
        if (mKeyPressed == PS3_L1 || mKeyPressed == PS3_L2 ||
                mKeyPressed == PS3_R1 || mKeyPressed == PS3_R2)
        {
            mc.modeStr = "torque";
            mc.value = 0.f;
            mc.waitTime = 0;
            publish(mc);
            mKeyPressed = PS3_NONE;
        }
        mKeyPressed = PS3_NONE;
    }
    else
    {
		for(size_t i =0; i < msg.buttons.size(); i++)
		{

			if(msg.buttons[i] == 0)
				continue;

			switch(i)
			{
			case PS3_X:

				if (mKeyPressed != PS3_NONE)
					break;

				// open the gripper
				mc.modeStr = "torque";
				mc.value = SOFT_MAX_TORQUE;
				mc.waitTime = GRIPPER_WAIT_TIME;

				publish(mc);

				mKeyPressed = PS3_X;
				break;

			case PS3_O:

				if (mKeyPressed != PS3_NONE)
					break;

				// close the gripper
				mc.modeStr = "torque";
				mc.value = -SOFT_MAX_TORQUE;
				mc.waitTime = GRIPPER_WAIT_TIME;

				publish(mc);

				mKeyPressed = PS3_O;
				break;

			case PS3_T:

				if (mKeyPressed != PS3_NONE)
					break;

				ROS_INFO("SENSOR TOGGLED");
				mSensor_pub.publish(empty_msg);

				mKeyPressed = PS3_T;
				break;

			case PS3_R1:
				ROS_INFO("SHOULDER BUTTON R1 %f", msg.axes[PS3_R1]);

				mc.modeStr = "torque";
				// interpolate between 0 and 0.05Nm

				mc.value = -SOFT_MAX_TORQUE * (float(msg.axes[PS3_R1]));
				mc.waitTime = 0;

				publish(mc);
				mKeyPressed = PS3_R1;
				break;

			case PS3_L1:
				ROS_INFO("SHOULDER BUTTON L1 %f", msg.axes[PS3_L1]);

				mc.modeStr = "torque";
				// interpolate between 0 and 0.05Nm
				mc.value = SOFT_MAX_TORQUE * float(msg.axes[PS3_L1]);
				mc.waitTime = 0;

				publish(mc);
				mKeyPressed = PS3_L1;
				break;

			case PS3_L2:
				mc.modeStr = "current";

				// interpolate between 0 and 0.5A
				mc.value = SOFT_MAX_CURRENT * float(msg.axes[PS3_L2]);
				mc.waitTime = 0;

				publish(mc);

				mKeyPressed = PS3_L2;
				break;

			case PS3_R2:
				mc.modeStr = "current";

				// interpolate between 0 and 0.5A
				mc.value = -SOFT_MAX_CURRENT * (float(msg.axes[PS3_R2]));
				mc.waitTime = 0;

				publish(mc);

				mKeyPressed = PS3_R2;
				break;

			default:
				break;
			}
		}
	}
}

int main(int argc, char **argv)
{
    // init ros and controller
    ros::init(argc, argv, "controller");
    PS3Controller controller;
    controller.init();
    ros::spin();
    return 0;
}
