#include <MotorHandler.h>

#define wheel_radius    0.1475
#define LEFT_MOTOR		106
#define RIGHT_MOTOR		107
#define UP              4
#define RIGHT           5
#define DOWN            6
#define L1				10
#define R1				11

/**
 * Called when a Twist message is received over the motor topic.
 */
void MotorHandler::moveCB(const geometry_msgs::Twist& msg)
{
	double left_vel = msg.linear.x - msg.angular.z * wheel_radius;
	double right_vel = msg.linear.x + msg.angular.z * wheel_radius;

	right_engine.setSpeed(right_vel);
	left_engine.setSpeed(left_vel);
}

double p_left, p_right = 0.01;
double i_left, i_right = 0;
double d_left, d_right = 0;
int ID;
int toSet = 0;

/**
 * Called when a Twist message is received over the motor topic.
*/
void MotorHandler::tweakCB(const mobile_base::tweak msg)
{
	double *p = ID == LEFT_MOTOR ? &p_left : &p_right;
	double *i = ID == LEFT_MOTOR ? &i_left : &i_right;
	double *d = ID == LEFT_MOTOR ? &d_left : &d_right;

	if(msg.data == UP)
	{
		switch(toSet){
		case 0:
			*p += 0.01;
			break;

		case 1:
			*i += 0.01;
			break;

		case 2:
			*d += 0.01;
			break;
		}
		if(ID == LEFT_MOTOR)
		{
			left_engine.setPID(*p, *i, *d);
			left_engine.printPID();
		}
			else
			{
				right_engine.setPID(*p, *i, *d);
				right_engine.printPID();
			}
	}

	if(msg.data == DOWN)
	{
		switch(toSet){
		case 0:
			if(*p > 0)
				*p -= 0.01;
			break;

		case 1:
			if(*i > 0)
				*i -= 0.01;
			break;

		case 2:
			if(*d > 0)
				*d -= 0.01;
			break;
		}
		if(ID == LEFT_MOTOR)
		{
			left_engine.setPID(*p, *i, *d);
			left_engine.printPID();
		}
		else
		{
			right_engine.setPID(*p, *i, *d);
			right_engine.printPID();
		}
	}

	if(msg.data == RIGHT){
		toSet = (toSet + 1)%3;

		if(toSet == 0)
			ROS_INFO("P");

		if(toSet == 1)
			ROS_INFO("I");

		if(toSet == 2)
			ROS_INFO("D");
	}

	if(msg.data == L1 || msg.data == R1){
		if(msg.motorID == LEFT_MOTOR){
			ROS_INFO("SWITCHED TO LEFT ENGINE");
			ID = LEFT_MOTOR;
		}
		else{
			ROS_INFO("SWITCHED TO RIGHT ENGINE");
			ID = RIGHT_MOTOR;
		}
	}
}

/**
 * Initalize MotorHandler and its attributes.
 */
void MotorHandler::init(char *path)
{
	twist_sub = nh_.subscribe("/movementTopic", 10, &MotorHandler::moveCB, this);
	tweak_sub = nh_.subscribe("/tweakTopic", 10, &MotorHandler::tweakCB, this);
	left_engine.init(path);
	right_engine.init(path);

	ROS_INFO("Initializing completed.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorHandler");

	ros::NodeHandle n;

	char *path=NULL;
	if (argc == 2)
		path = argv[1];

	MotorHandler motorHandler;
	motorHandler.init(path);

	ros::spin();

	return 0;
}

