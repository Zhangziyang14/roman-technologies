#include <BaseController.h>

char *path=NULL;

/**
 * Initalize the attributes of the controller
*/
void BaseController::init()
{
    // intialize subscribers
    mKey_sub = mNodeHandle.subscribe("joy", 10, &BaseController::keyCB, this);
    mTwist_pub = mNodeHandle.advertise<geometry_msgs::Twist>("/movementTopic", 10);

    mTweak_pub = mNodeHandle.advertise<mobile_base::tweak>("/tweakTopic", 10);
    ROS_INFO("BaseController initialized");
}

void BaseController::keyCB(const sensor_msgs::Joy& msg){

    geometry_msgs::Twist twist_msg;
    mobile_base::tweak button;
    
    //No button is pressed, so sum of vector is zero -> stand still
    if(std::accumulate(msg.buttons.begin(), msg.buttons.end(), 0) == 0){

        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
        mTwist_pub.publish(twist_msg);
        mKeyPressed = PS3_NONE;
    }
    else{
	    for(size_t i = 0; i < msg.buttons.size(); i++){

	    if(msg.buttons[i] == 0)
		    continue;

	        switch(i){
            //Accelerate when X button is pressed
	        case PS3_X:
                    twist_msg.linear.x = -2 * M_PI * float(msg.axes[PS3_X]);
                    ROS_INFO("DEBUG %f", float(msg.axes[PS3_X]));
                    
                    if(msg.axes[PS3_LEFT_HORIZONTAL]){
                        twist_msg.angular.z = 4.f * float(msg.axes[PS3_LEFT_HORIZONTAL]);
                        ROS_INFO("DEBUG %f", 4.f * float(msg.axes[PS3_LEFT_HORIZONTAL]));
                    }
                    mTwist_pub.publish(twist_msg);
	        break;
	       
            //Brake if O button has been pressed
            case PS3_O:            
                twist_msg.linear.x = 0;
                twist_msg.angular.z = 0;
                mTwist_pub.publish(twist_msg);
            break;

            case PS3_RIGHT:
                 if(mKeyPressed != PS3_NONE)
                    break;
                 
                 button.data = PS3_RIGHT;
                 mTweak_pub.publish(button);
                 mKeyPressed = PS3_RIGHT;
            break;

            case PS3_UP:
                if(mKeyPressed != PS3_NONE)
                    break;

                button.data = PS3_UP;
                mTweak_pub.publish(button);
                mKeyPressed = PS3_UP;
            break;

            case PS3_DOWN:
                if(mKeyPressed != PS3_NONE)
                    break;

                button.data = PS3_DOWN;
                mTweak_pub.publish(button);
                mKeyPressed = PS3_DOWN;
            break;

            case PS3_L1:
                if(mKeyPressed != PS3_NONE)
                    break;

                    button.data = PS3_L1;
                    button.motorID = 106;
                    mTweak_pub.publish(button);
                    mKeyPressed = PS3_L1;
            break;

            case PS3_R1:
                if(mKeyPressed != PS3_NONE)
                    break;

                button.data = PS3_R1;
                button.motorID = 107;
                mTweak_pub.publish(button);
                mKeyPressed = PS3_R1;
            break;

            default:
                break;
            }
	        usleep(1000000);
        }
    }
}

int main(int argc, char **argv)
{
    // init ros and controller
    ros::init(argc, argv, "controller");
    BaseController base_controller;

    if (argc == 2)
        path = argv[1];

    base_controller.init();
    ros::spin();

    return 0;
}
