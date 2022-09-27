#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>
#include <roboteq_driver/channel_values.h>
#include <roboteq_driver/command_srv.h>

namespace roboteeq_odometry
{
class RoboteqOdometry
{
public:

     RoboteqOdometry(ros::NodeHandle& nh_local);
    ~RoboteqOdometry();
    void spin();

private:

    void initialize();
    void update();
    void encoderCb(const roboteq_driver::channel_values& ticks);

    ros::NodeHandle nh_local_;
    ros::ServiceClient command_client;
	ros::Subscriber wheels_sub;
    ros::Publisher odom_pub;

    ros::Duration t_delta;
	ros::Time t_next;
	ros::Time then;
    ros::Time current_time, last_time;

	roboteq_driver::command_srv command_service;

    tf::TransformBroadcaster odom_broadcaster;

    int ppr;
    double encoder_min;
	double encoder_max;
	double encoder_low_wrap;
	double encoder_high_wrap;
	double prev_lencoder;
	double prev_rencoder;
	double lmult;
	double rmult;
	double left;
	double right;
	double rate;
    double enc_left ;
	double enc_right;
	double ticks_meter;
	double base_width;
	double dx;
	double dr;
	double x_final,y_final, theta_final;
	double radius, gear_ratio;
    bool publish_tf;
    bool get_odom;
    std::string encoder_topic;
	std::string odom_frame;
	std::string base_frame;
	std::string command_srv;
};
}