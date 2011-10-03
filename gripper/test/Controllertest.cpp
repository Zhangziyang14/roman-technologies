#include "roman/Controller.h"
#include <gtest/gtest.h>

ros::Publisher keyTest_pub;
ros::Subscriber sensorTest_sub;
bool called = false;

void activateSensorCBTest(const std_msgs::Empty::ConstPtr& msg){
    called = true;
}

///Test whether the sensor will be activated when triangle key is toggled
TEST(SensorCallBackTest, SensorActivated)
{
    roman::Key msg;
    //Triangle button is pressed (button no. 12)
    msg.keys.push_back(12);
    msg.values.push_back(255);

    keyTest_pub.publish(msg);
    ros::Rate rate(10);

        unsigned int cycles = 10;
        while(cycles-- && !called){
            ros::spinOnce();
            rate.sleep();
        }

    EXPECT_TRUE(called);
    called = false;
}

//Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ros::init(argc, argv, "controllertest");

    ros::NodeHandle node;
    keyTest_pub     = node.advertise<roman::Key>("keyTopic", 10);
    sensorTest_sub  = node.subscribe("sensorTopic", 10, activateSensorCBTest);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
