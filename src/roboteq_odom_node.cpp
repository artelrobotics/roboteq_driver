#include "roboteq_driver/roboteq_odom.h"

using namespace roboteeq_odometry;

int main(int argc, char **argv)

{
	ros::init(argc, argv,"diff_odom");
    ros::NodeHandle nh_local("~");
	RoboteqOdometry odometry(nh_local);
	odometry.spin();
	return 0;
}