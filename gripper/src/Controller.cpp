#include "gripper/Controller.h"

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
 * Initalise the attributes of the controller
*/
void Controller::init()
{
    // intialise publishers
    mMotor_pub   = mNodeHandle.advertise<nero_msgs::GripperControl>("/gripperMotorTopic", 10);
    mJoint_pub   = mNodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 10);
    mGripper_pub = mNodeHandle.advertise<std_msgs::UInt8>("/gripper_state", 1);

	// initialize subscribers
    mOpen_sub	= mNodeHandle.subscribe("/cmd_gripper_state", 1, &Controller::commandCB, this);

    ROS_INFO("Controller initialised");
}

void Controller::commandCB(const std_msgs::Bool& msg)
{
	setGripper(msg.data, true);
}

/**
  * Publish the given MotorControl message if possible and change visualization in rviz
 */
void Controller::publish(nero_msgs::GripperControl mc)
{
    if (mc.mode == CM_NONE)
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

        // publish the message to the motor handler
        mMotor_pub.publish(mc);
    }
}
