#include "Controller.h"

#define JOINT_COUNT 7

/// names of the joints in the RViz visualization
static const char *jointNames[JOINT_COUNT] =
{
    "base_to_left_finger",
    "left_finger_to_left_finger_top",
    "right_palm_to_right_upper_finger",
    "right_palm_to_right_lower_finger",
    "right_upper_finger_to_right_upper_finger_top",
    "right_lower_finger_to_right_lower_finger_top",
    "base_to_grab_object",
};

/// modifications in yaw rotation for the joints when the gripper is closed
static const float jointYaw[JOINT_COUNT] =
{
    -0.785f,
    -1.04f,
    0.785f,
    0.785f,
    1.04f,
    1.04f,
    -0.375f,
};

/**
 * Sends yaw values for joints. Takes mGripperState into consideration for chosing the yaw value.
*/
void Controller::UpdateJoints()
{
    sensor_msgs::JointState msg;

    // not sure why this is needed, but it won't work without it
    msg.header.stamp = ros::Time::now();

    // JointState message contains vectors with the joint name and the yaw value
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        msg.name.push_back(jointNames[i]);
        msg.position.push_back(mGripperState == GS_CLOSED ? jointYaw[i] : 0.f);
    }

    mJoint_pub.publish(msg);
    ROS_INFO("Changed joints");
}

/**
 * Initalize the attributes of the controller
*/
void Controller::init()
{
    // intialize publishers
    mSensor_pub  = mNodeHandle.advertise<std_msgs::Empty>("sensorTopic", 10);
    mMotor_pub   = mNodeHandle.advertise<gripper::MotorControl>("motorTopic", 10);
    mJoint_pub   = mNodeHandle.advertise<sensor_msgs::JointState>("joint_states", 10);

    // initialize subscribers
    mSensor_sub = mNodeHandle.subscribe("sensorFeedbackTopic", 10, &Controller::readSensorDataCB, this);
    mKey_sub    = mNodeHandle.subscribe("joy", 10, &Controller::keyCB, this);

    ROS_INFO("Controller initialized");
}

/**
 * Called when new sensor data is made available.
*/
void Controller::readSensorDataCB(const gripper::DistancePtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->distance);

    // default message, open the gripper
    gripper::MotorControl mc;
    mc.modeStr = "torque";
    mc.value = 0.02f;
    mc.waitTime = 1000;

    // Is the distance smaller than 10cm and are we not grabbing the object yet? Then close the gripper
    if (msg->distance < 100 && mGripperState != GS_CLOSED)
    {
        mc.value = -mc.value;
        mMotor_pub.publish(mc);
        mGripperState = GS_CLOSED;
    }
    // Is the distance greater than 15cm and are we grabbing the object, then open the gripper.
    else if (msg->distance > 150 && mGripperState != GS_OPEN)
    {
        mMotor_pub.publish(mc);
        mGripperState = GS_OPEN;
    }
}

/**
  * Publish the given MotorControl message if possible and change visualization in rviz
 */
void Controller::publish(gripper::MotorControl mc){
    if (mc.modeStr.empty() == false)
    {
        if (mGripperState != GS_OPEN && mc.value > 0.f)
        {
            // we are opening the gripper
            mGripperState = GS_OPEN;
            UpdateJoints();
        }
        else if (mGripperState != GS_CLOSED && mc.value < 0.f)
        {
            // we are closing the gripper
            mGripperState = GS_CLOSED;
            UpdateJoints();
        }

        // print the message for debugging
        std::stringstream ss("");
        ss << mc.modeStr << " " << mc.value << " " << mc.waitTime;
        ROS_INFO("message: %s", ss.str().c_str());

        // publish the message to the motor handler
        mMotor_pub.publish(mc);
    }
}


/**
 * Called when sensor should be activated/deactivated.
*/
void Controller::keyCB(const sensor_msgs::Joy& msg)
{
    //ROS_INFO("Key Received. Id = %d, value = %d", msg->keys[i], msg->values[i]);

    gripper::MotorControl mc;
    std_msgs::Empty empty_msg;

    //No button is pressed, so sum of vector is zero
    if(std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0){
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
    else{
	for(size_t i =0; i < msg.buttons.size(); i++){

	if(msg.buttons[i] == 0)
		continue;

	switch(i){
	case PS3_X:

	if (mKeyPressed != PS3_NONE)
        	break;

            // open the gripper
            mc.modeStr = "torque";
            mc.value = 0.5f;
            mc.waitTime = 1000;

            publish(mc);

	    mKeyPressed = PS3_X;
	    break;

	case PS3_O:

	   if (mKeyPressed != PS3_NONE)
	           break;

	    // close the gripper
            mc.modeStr = "torque";
            mc.value = -0.5f;
            mc.waitTime = 1000;

            publish(mc);
	    
	    mKeyPressed = PS3_O;
	    break;
	
	case PS3_T:

	   if (mKeyPressed != PS3_NONE)
           	break;

            ROS_INFO("SENSOR TOGGLED");
            mSensor_pub.publish(empty_msg);

            mKeyPressed = PS3_T;

	    mKeyPressed = PS3_T;
	    break;

	case PS3_R1:	
            ROS_INFO("SHOULDER BUTTON R1 %f", msg.axes[PS3_R1]);

            mc.modeStr = "torque";
            // interpolate between 0 and 0.05Nm

            mc.value = -0.05f * (float(msg.axes[PS3_R1]));
            mc.waitTime = 0;

            publish(mc);
            mKeyPressed = PS3_R1;
	    break;
	
	case PS3_L1:
            ROS_INFO("SHOULDER BUTTON L1 %f", msg.axes[PS3_L1]);

            mc.modeStr = "torque";
            // interpolate between 0 and 0.05Nm
            mc.value = 0.05f * float(msg.axes[PS3_L1]);
            mc.waitTime = 0;

            publish(mc);	
	    mKeyPressed = PS3_L1;
	    break;

	case PS3_L2:
	    mc.modeStr = "current";
            // interpolate between 0 and 0.5A

            mc.value = 0.5f * float(msg.axes[PS3_L2]);
            mc.waitTime = 0;

            publish(mc);

            mKeyPressed = PS3_L2;
	    break;

	case PS3_R2:
	    mc.modeStr = "current";
            // interpolate between 0 and 0.5A

            mc.value = -0.5f * (float(msg.axes[PS3_R2]));
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

int main(int argc, char **argv){
    // init ros and controller
    ros::init(argc, argv, "controller");
    Controller controller;
    controller.init();
    ros::spin();
    return 0;
}
