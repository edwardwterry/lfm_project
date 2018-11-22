/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>	   
 */
#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/SwiftproState.h>
#include <swiftpro/status.h>
#include <swiftpro/position.h>
#include <swiftpro/angle4th.h>

serial::Serial _serial;				// serial object
swiftpro::SwiftproState pos;
float position[4] = {0.0};			// 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
char  strdata[2048];				// global variables for handling string

/* 
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::position& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char x[10]; // all were 10
	char y[10];
	char z[10];

	pos.x = msg.x;
	pos.y = msg.y;
	pos.z = msg.z;
	sprintf(x, "%.2f", msg.x);
	sprintf(y, "%.2f", msg.y);
	sprintf(z, "%.2f", msg.z);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
	// std::string Gcode1 = (std::string)"G0 X" + x;
	// std::string Gcode2 = (std::string)"G0 Y" + y;
	// std::string Gcode3 = (std::string)"G0 Z" + z;
	// Gcode4 = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";

	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());

	// _serial.write(Gcode1.c_str());
	// result.data = _serial.read(_serial.available());
	// _serial.write(Gcode2.c_str());
	// result.data = _serial.read(_serial.available());
	// _serial.write(Gcode3.c_str());
	// result.data = _serial.read(_serial.available());
	// _serial.write(Gcode.c_str());
	// result.data = _serial.read(_serial.available());

	// ROS_INFO_STREAM("Read:" << result.data);
}


/* 
 * Description: callback when receive data from angle4th_topic
 * Inputs: 		msg(float)			angle of 4th motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle4th_callback(const swiftpro::angle4th& msg)
{
	std::string Gcode = "";
	std_msgs::String result;
	char m4[10];
	
	pos.motor_angle4 = msg.angle4th;
	sprintf(m4, "%.2f", msg.angle4th);
	Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs: 		msg(uint8)			status of gripper: attach if 1; detach if 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M17\r\n";   // attach
	else if (msg.status == 0)
		Gcode = (std::string)"M2019\r\n";
	else
	{
		ROS_INFO("Error:Wrong swiftpro status input");
		return;
	}
	
	pos.swiftpro_status = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from gripper_topic
 * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void gripper_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M2232 V1" + "\r\n";
	else if (msg.status == 0)
		Gcode = (std::string)"M2232 V0" + "\r\n";
	else
	{
		ROS_INFO("Error:Wrong gripper status input");
		return;
	}
	
	pos.gripper = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from pump_topic
 * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void pump_callback(const swiftpro::status& msg)
{
	std::string Gcode = "";
	std_msgs::String result;

	if (msg.status == 1)
		Gcode = (std::string)"M2231 V1" + "\r\n";
	else if (msg.status == 0)
		Gcode = (std::string)"M2231 V0" + "\r\n";
	else
	{
		ROS_INFO("Error:Wrong pump status input");
		return;
	}
	
	pos.pump = msg.status;
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}

void handlestr()
{
	char  *pch = strtok(strdata, " ");
	float value[8];
	int   index = 0;

	while (pch != NULL && index < 5)
	{
		value[index] = atof(pch+1);
		pch = strtok(NULL, " ");
		index++;
	}
	position[0] = value[1];
	position[1] = value[2];
	position[2] = value[3];
	position[3] = value[4];
}


void handlechar(char c)
{
	static int index = 0;

	switch(c)
	{
		case '\r':
			break;	

		case '\n':
			strdata[index] = '\0';
			handlestr();
			index = 0;
			break;

		default:
			strdata[index++] = c;
			break;
	}
}

/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{	
	ros::init(argc, argv, "swiftpro_write_node");
	ros::NodeHandle nh;
	swiftpro::SwiftproState swiftpro_state;

	std_msgs::String result;

	ros::Subscriber sub1 = nh.subscribe("position_write_topic", 1, position_write_callback);
	ros::Subscriber sub2 = nh.subscribe("swiftpro_status_topic", 1, swiftpro_status_callback);
	ros::Subscriber sub3 = nh.subscribe("angle4th_topic", 1, angle4th_callback);
	ros::Subscriber sub4 = nh.subscribe("gripper_topic", 1, gripper_callback);
	ros::Subscriber sub5 = nh.subscribe("pump_topic", 1, pump_callback);
	ros::Publisher 	 pub = nh.advertise<swiftpro::SwiftproState>("SwiftproState_topic", 1);
	ros::Rate loop_rate(20);

	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		ros::Duration(3.5).sleep();				// wait 3.5s
		// _serial.write("M2120 V0\r\n");			// stop report position
		_serial.write("M2120 V0.05\r\n");			// stop report position
		ros::Duration(0.1).sleep();				// wait 0.1s
		_serial.write("M17\r\n");				// attach
		ros::Duration(0.1).sleep();				// wait 0.1s
		ROS_INFO_STREAM("Attach and wait for commands");
	}

	while (ros::ok())
	{
		if (_serial.available())
		{
			result.data = _serial.read(_serial.available());
			// ROS_INFO_STREAM("Read:" << result.data);
			for (int i = 0; i < result.data.length(); i++)
				handlechar(result.data.c_str()[i]);

			swiftpro_state.pump = 0;
			swiftpro_state.gripper = 0;
			swiftpro_state.swiftpro_status = 0;
			swiftpro_state.motor_angle1 = 0.0;
			swiftpro_state.motor_angle2 = 0.0;
			swiftpro_state.motor_angle3 = 0.0;
			swiftpro_state.motor_angle4 = position[3];
			swiftpro_state.x = position[0];
			swiftpro_state.y = position[1];
			swiftpro_state.z = position[2];
			pub.publish(swiftpro_state);
			// ROS_INFO("position: %.2f %.2f %.2f %.2f", position[0], position[1], position[2], position[3]);
		}
		// pub.publish(pos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


