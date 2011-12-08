#include "gripper/TemperatureListener.h"

/**
 * Initalize the attributes of TemperatureListener
*/
void TemperatureListener::init()
{
    mTempSensor_sub = mNodeHandle.subscribe("temperatureFeedbackTopic", 10, &TemperatureListener::readTemperatureSensorDataCB, this);

    mStartTime = time(NULL);
    mLastMeasureTime = mStartTime;
    mTemperatureFile.open("/tmp/temperature.txt");
    memset(mTemperatures, 0, sizeof(int) * 30);
    mTemperatureIndex = 0;

    ROS_INFO("TemperatureListener initialized");
}

/**
 * Called when new sensor data is made available.
*/
void TemperatureListener::readTemperatureSensorDataCB(const gripper::TemperaturePtr& msg)
{
    mTemperatures[mTemperatureIndex] = msg->temperature;
    mTemperatureIndex = (mTemperatureIndex + 1) % 30;

    // only save temperature every 15 seconds
    if (time(NULL) - mLastMeasureTime < 15)
        return;

    int avgTemperature = 0;
    for (int i = 0; i < 30; i++)
        avgTemperature += mTemperatures[i];
    
    mLastMeasureTime = time(NULL);
    ROS_INFO("Temperatuur is: %d", avgTemperature);
    mTemperatureFile << "time=" << mLastMeasureTime - mStartTime << " temperature=" << avgTemperature << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temperature");
    TemperatureListener temp_listener;
    temp_listener.init();
    ros::spin();
    return 0;
}
