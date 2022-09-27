#include "roboteq_driver/roboteq_driver.h"

using namespace roboteq_driver;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
	RoboteqDriver driver(nh, nh_local);
	ros::waitForShutdown();
	return 0;
}