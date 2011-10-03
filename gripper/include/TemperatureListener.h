#ifndef __ULTRASONE_H
#define __ULTRASONE_H

#include <ros/ros.h>
#include "Temperature.h"
#include <fstream>
#include <time.h>

/// Reads temperature sensor and stops the motor when overheated
class TemperatureListener
{
protected:
    ros::NodeHandle mNodeHandle;        /// ROS node handle

    ros::Subscriber mTempSensor_sub;    /// Subscriber for temperature feedback topic, receives temperatures

    time_t mStartTime;                  /// Start time of this node, used for saving time stamp of temperature
    time_t mLastMeasureTime;            /// Time when last temperature measurement was done, used for temperature saving interval
    std::ofstream mTemperatureFile;     /// Stream for the temperature file
    int mTemperatures[30];              /// Array of temperature samples to smooth the output
    int mTemperatureIndex;              /// Current index of the temperature sample array to be modified
public:
    /// Constructor
    TemperatureListener() : mNodeHandle(""){ }

    /// Destructor
    ~TemperatureListener()
    {
        mTemperatureFile.close();
        mNodeHandle.shutdown();
    }

    /// Initialize node
    void init();

    void readTemperatureSensorDataCB(const gripper::TemperaturePtr& msg);
};

#endif /* __CONTROLLER_H */
