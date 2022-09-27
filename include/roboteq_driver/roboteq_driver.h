#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <typeinfo>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_driver/channel_values.h>
#include <roboteq_driver/config_srv.h>
#include <roboteq_driver/command_srv.h>
#include <roboteq_driver/maintenance_srv.h>
#include <roboteq_driver/emergency_stop_srv.h>
#include <roboteq_driver/safety_stop_srv.h>

namespace roboteq_driver
{
class RoboteqDriver
{
public:
    RoboteqDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~RoboteqDriver();

private:
    void initialize();
    void connect();
    void cmd_vel_callback(const geometry_msgs::Twist &msg);
    double calculate_right_speed(double x, double z);
    double calculate_left_speed(double x, double z);
    double to_rpm(double value);
    double max_limit(double speed);
    bool configservice(roboteq_driver::config_srv::Request &request, roboteq_driver::config_srv::Response &response);
    bool commandservice(roboteq_driver::command_srv::Request &request, roboteq_driver::command_srv::Response &response);
    bool multicommandservice(roboteq_driver::command_srv::Request &request, roboteq_driver::command_srv::Response &response);
    bool maintenanceservice(roboteq_driver::maintenance_srv::Request &request, roboteq_driver::maintenance_srv::Response &response);
	bool emergencystopservice(roboteq_driver::emergency_stop_srv::Request &request, roboteq_driver::emergency_stop_srv::Response &response);
	bool safetystopservice(roboteq_driver::safety_stop_srv::Request &request, roboteq_driver::safety_stop_srv::Response &response);
    void run();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::Publisher read_publisher;
	ros::Subscriber cmd_vel_sub;
    ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer multicommandsrv;
	ros::ServiceServer maintenancesrv;
	ros::ServiceServer emergencysrv;
	ros::ServiceServer safetystopsrv;

    serial::Serial ser;
	std::string port;
	int32_t baud;

    typedef std::string Key;
    typedef std::string Val;
    std::map<Key, Val> map_sH;
    
    int frequency;
	int pub_rate;
	double wheelbase;
	double radius;
	double gearRatio;
	double maxRPM;

};
}