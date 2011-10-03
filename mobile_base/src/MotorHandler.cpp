#include <MotorHandler.h>

/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
   //Magic formula
}

/**
 * Initalize MotorHandler and its attributes.
*/
void MotorHandler::init(char *path)
{
    twist_sub = nh_.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);

    left_engine.init(path);
    right_engine.init(path);

    ROS_INFO("Initializing completed."); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    char *path=NULL;
    if (argc == 2)
        path = argv[1];
 
    MotorHandler motorHandler;
    motorHandler.init(path);

    ros::spin();

    return 0;
}

