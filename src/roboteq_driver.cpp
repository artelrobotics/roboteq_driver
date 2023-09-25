#include "roboteq_driver/roboteq_driver.h"

using namespace roboteq_driver;

RoboteqDriver::RoboteqDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local){
    initialize();
}


RoboteqDriver::~RoboteqDriver(){
    if (ser.isOpen())
		{
			ser.close();
		}
}


void RoboteqDriver::connect()
	{
		try
		{
			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
		}
		if (ser.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			ROS_INFO_STREAM("Serial Port is not open");
		}
		run();
	}

void RoboteqDriver::initialize(){
    nh_local_.param<std::string>("port", port, "/dev/ttyACM0");
    nh_local_.param("baud", baud, 115200);
    nh_local_.param("wheelbase", wheelbase, 1.0);
    nh_local_.param("radius", radius, 1.0);
    nh_local_.param("gear_ratio", gearRatio, 1.0);
    nh_local_.param("max_rpm", maxRPM, 1000.0);
    
    nh_local_.getParam("frequency", frequency);
    nh_local_.getParam("query", map_sH);
    nh_local_.getParam("pub_rate", pub_rate);

    configsrv = nh_local_.advertiseService("config_service", &RoboteqDriver::configservice, this);
    commandsrv = nh_local_.advertiseService("command_service", &RoboteqDriver::commandservice, this);
    multicommandsrv = nh_local_.advertiseService("dualchannel_command_service", &RoboteqDriver::multicommandservice, this);
    maintenancesrv = nh_local_.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
    emergencysrv = nh_local_.advertiseService("emergency_stop_service", &RoboteqDriver::emergencystopservice, this);
    safetystopsrv = nh_local_.advertiseService("safety_stop_service", &RoboteqDriver::safetystopservice, this);

    cmd_vel_sub = nh_.subscribe("cmd_vel", 10, &RoboteqDriver::cmd_vel_callback, this);
    connect();
}


bool RoboteqDriver::configservice(roboteq_driver::config_srv::Request &request, roboteq_driver::config_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}

bool RoboteqDriver::commandservice(roboteq_driver::command_srv::Request &request, roboteq_driver::command_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}

bool RoboteqDriver::multicommandservice(roboteq_driver::command_srv::Request &request, roboteq_driver::command_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "!" << request.userInput << " " << "1" << " " << request.value << "_" << "!" << request.userInput << " " << "2" << " " << request.value << "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}

bool RoboteqDriver::maintenanceservice(roboteq_driver::maintenance_srv::Request &request, roboteq_driver::maintenance_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		str << "%" << request.userInput << " "
			<< "_";
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}

bool RoboteqDriver::emergencystopservice(roboteq_driver::emergency_stop_srv::Request &request, roboteq_driver::emergency_stop_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		if (request.state)
		{
			str << "!" << "EX" << "_";
		}
		else
		{
			str << "!" << "MG" << "_";
		}
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}

bool RoboteqDriver::safetystopservice(roboteq_driver::safety_stop_srv::Request &request, roboteq_driver::safety_stop_srv::Response &response)
	{
		int res = 0;
		std::stringstream str;
		if (request.state)
		{
			str << "!" << "SFT 1" << "_" << "!" << "SFT 2" << "_";
		}
		else
		{
			for (int i=0; i<10; i++)
			{
				str << "!S 1 0" << "_" << "!S 2 0" << "_";
				sleep(0.1);
			}
		}
		res = ser.write(str.str());
		response.result = (res == str.str().size());
		ser.flush();
		return true;
	}


void RoboteqDriver::cmd_vel_callback(const geometry_msgs::Twist &msg)
	{
		std::stringstream mcommands;
		mcommands << "!S 1"
				<< " " << to_rpm(calculate_right_speed(msg.linear.x, msg.angular.z)) << "_"
				<< "!S 2"
				<< " " << to_rpm(calculate_left_speed(msg.linear.x, msg.angular.z)) << "_";
		ser.write(mcommands.str());
		ser.flush();
	}

double RoboteqDriver::calculate_right_speed(double x, double z)
	{
		return (2 * x - z * wheelbase) / (2 * radius);
	}

double RoboteqDriver::calculate_left_speed(double x, double z)
	{
		return (2 * x + z * wheelbase) / (2 * radius);
	}

double RoboteqDriver::to_rpm(double value)
	{

		return max_limit((value * 60 * gearRatio) / (2 * M_PI));
	}

double RoboteqDriver::max_limit(double speed)
	{
		if (speed > 0)
		{
			return std::min<double>(maxRPM, speed);
		}
		else return std::max<double>(-maxRPM, speed);
	}


void RoboteqDriver::run()
	{
		std::stringstream ss0;
		std::stringstream ss1;
		std::stringstream ss2;
		std::stringstream ss3;
		std::vector<std::string> KH_vector;

		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
		{
			Key KH = iter->first;

			KH_vector.push_back(KH);

			Val VH = iter->second;

			ss1 << VH << "_";
		}
		ss1 << "# " << frequency << "_";
		std::vector<ros::Publisher> publisherVecH;
		for (int i = 0; i < KH_vector.size(); i++)
		{
			publisherVecH.push_back(nh_local_.advertise<roboteq_driver::channel_values>(KH_vector[i], 100));
		}
		ser.write(ss0.str());
		ser.write(ss1.str());
		ser.write(ss2.str());
		ser.write(ss3.str());

		ser.flush();
		int count = 0;
		read_publisher = nh_local_.advertise<std_msgs::String>("read", 1000);
		sleep(2);
		ros::Rate loop_rate(pub_rate);
		while (ros::ok())
		{

			ros::spinOnce();
			if (ser.available())
			{

				std_msgs::String result;
				result.data = ser.read(ser.available());

				read_publisher.publish(result);
				boost::replace_all(result.data, "\r", "");
				boost::replace_all(result.data, "+", "");

				std::vector<std::string> fields;

				std::vector<std::string> Field9;
				boost::split(fields, result.data, boost::algorithm::is_any_of("D"));

				std::vector<std::string> fields_H;
				boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));

				if (fields_H[0] == "H")
				{

					for (int i = 0; i < publisherVecH.size(); ++i)
					{

						std::vector<std::string> sub_fields_H;

						boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
						roboteq_driver::channel_values Q1;

						for (int j = 0; j < sub_fields_H.size(); j++)
						{

							try
							{
								Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
							}
							catch (const std::exception &e)
							{
								count++;
								if (count > 10)
								{
									ROS_INFO_STREAM("Garbage data on Serial");
								}
							}
						}

						publisherVecH[i].publish(Q1);
					}
				}
			}
			loop_rate.sleep();
		}
	}