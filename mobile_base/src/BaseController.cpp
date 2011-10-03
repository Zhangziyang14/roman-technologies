#include <BaseController.h>

char *path=NULL;

/**
 * Initalize the attributes of the controller
*/
void BaseController::init()
{
    // intialize subscribers
    leftEngine  = new Motor(106);
    rightEngine = new Motor(107);
    armEngine   = new Motor(108);

    leftEngine->init(path);
    rightEngine->init(path);
    armEngine->init(path);

    ROS_INFO("BaseController initialized");
}

/**
 * Drives the base forward
*/
  void BaseController::moveForward(double speed){
      leftEngine->setSpeed(speed);
      rightEngine->setSpeed(speed);
  }

  /**
   * Drives the base backward
  */
  void BaseController::moveBackward(double speed){
      leftEngine->setSpeed(-speed);
      rightEngine->setSpeed(-speed);
  }

  /**
   * Turns the base to the left
  */
  void BaseController::turnLeft(double speed){
      leftEngine->setSpeed(-speed);
      rightEngine->setSpeed(speed);
  }

  /**
   * Turns the base to the right
  */
  void BaseController::turnRight(double speed){
      leftEngine->setSpeed(speed);
      rightEngine->setSpeed(-speed);
  }

  /**
   * Stops the base
  */
  void BaseController::brake(){
      leftEngine->setSpeed(0);
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
