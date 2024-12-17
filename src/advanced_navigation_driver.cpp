/****************************************************************/
/*                                                              */
/*                    Advanced Navigation                       */
/*         		  ROS2 Driver			        */
/*          Copyright 2020, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include "NTRIP_Client/NTRIP/ntripclient.h"
#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <GeographicLib/LocalCartesian.hpp>

#define RADIANS_TO_DEGREES (180.0/M_PI)
const double PI = 4*atan(1);

#define USE_FLU true
#define USE_ENU true
#define MAG_DEC 0.22008602 //zzCJ: Set to 0.0 after testing assuming the certus outputs true north... 

//RCLCPP_WARN(node->get_logger(), "WARNING MSG PRINT");
//RCLCPP_INFO(node->get_logger(), "INFORMATION MSG PRINT");
//RCLCPP_ERROR(node->get_logger(), "ERROR MSG PRINT");

#include <iostream>
#include <deque>
#include <cmath>

class RollingStdDev {
public:
    RollingStdDev(size_t window_size) : window_size(window_size), sum(0), sum_sq(0) {}

    void add_value(double value) {
        values.push_back(value);
        sum += value;
        sum_sq += value * value;

        if (values.size() > window_size) {
            double old_value = values.front();
            values.pop_front();
            sum -= old_value;
            sum_sq -= old_value * old_value;
        }
    }

    double mean() const {
        return sum / values.size();
    }

    double standard_deviation() const {
        if (values.size() < 2) return 0.0; // not enough values to compute standard deviation
        double mean_val = mean();
        double variance = (sum_sq / values.size()) - (mean_val * mean_val);
        return std::sqrt(variance);
    }

	private:
		std::deque<double> values;
		size_t window_size;
		double sum;
		double sum_sq;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("AdNav_Node");
	
	//For Debugging
	// RCLCPP_INFO(node->get_logger(), "argc: %d\n", argc);
	// for(int i = 0; i < argc; i++){
	// 	RCLCPP_INFO(node->get_logger(), "argv[%d}: %s\n", i, argv[i]);
	// }
	RollingStdDev rstd_acc_x(1000);
	RollingStdDev rstd_acc_y(1000); 
	RollingStdDev rstd_acc_z(1000); 
	RollingStdDev rstd_gyr_x(1000); 
	RollingStdDev rstd_gyr_y(1000); 
	RollingStdDev rstd_gyr_z(1000); 

	GeographicLib::LocalCartesian geo_converter;
	bool initENU = false;

    // Define the specific value to match
    const char* specific_value = "--ros-args";

    // Check if there are any arguments and if the last argument matches the specific value
    if (argc > 1 && strcmp(argv[argc - 1], specific_value) == 0) {
        // Reduce argc by one to exclude the last argument
        argc -= 1;
    }

	if(argc == 1){
		printf("usage: ros2 run package_name executable_name [baud_rate] [comm_port]\npackage_name     Name of the ROS package\nexecutable_name  Name of the executable\nbaud_rate        The Baud rate configured on the device. Default 115200\ncomm_port        The COM port of the connected device. Default /dev/ttyUSB0\n");
		exit(EXIT_FAILURE);	
	}
	else{
		RCLCPP_INFO(node->get_logger(), "Your Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");  
	}

	// Set up for Log File
	static const uint8_t request_all_configuration[] = { 0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7 };
	FILE *log_file;
	char filename[32];
	time_t rawtime;
	struct tm * timeinfo;
	// int write_counter = 0;

	// Set up the COM port
	int baud_rate;
	std::string com_port;

	// String ID for all publishers
	std::stringstream gnssFixType;
	tf2::Quaternion orientation;
	std_msgs::msg::String gnss_fix_type_msgs;

  	// NTRIP Varialbles	
	int error = 0;
	int bytes_received;
	int numbytes = 0;
	int remain = numbytes;
	int pos = 0;
	int state; // 0 = NTRIP Info provided, 1 = No Arguement, 3 = Baudrate and Port for serial 
	struct Args args;
	char buf[MAXDATASIZE];

  	// Configuring based on the state, what sort of driver to run
	if (argc == 1){
		printf("usage: ros2 run package_name executable_name [baud_rate] [comm_port]\npackage_name     Name of the ROS package\nexecutable_name  Name of the executable\nbaud_rate        The Baud rate configured on the device. Default 115200\ncomm_port        The COM port of the connected device. Default /dev/ttyUSB0\n");
		exit(EXIT_FAILURE);	
		//com_port = std::string("/dev/ttyUSB0");  
    	//baud_rate = 115200;
		//state = 1;
	}
	else if (argc == 3) {
		com_port = std::string(argv[2]);
		baud_rate = atoi(argv[1]);
		state = 3;
	}
	else{
		getargs(argc, argv, &args);
		state = 0;
	}
  
	// Creating the ROS2 Publishers
	auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
	auto nav_sat_fix_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

	rclcpp::Rate loop_rate(10);

	// IMU sensor_msgs/Imu
	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.stamp.sec = 0;
	imu_msg.header.stamp.nanosec = 0;
	imu_msg.header.frame_id = "imu";
	imu_msg.orientation.x = 0.0;
	imu_msg.orientation.y = 0.0;
	imu_msg.orientation.z = 0.0;
	imu_msg.orientation.w = 0.0;
	imu_msg.orientation_covariance = {	pow(0.0017, 2.0f), 0.0, 0.0,
										0.0, pow(0.0017, 2.0f), 0.0, 
										0.0, 0.0, pow(0.0017, 2.0f)}; // fixed
	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = 0.0;
	imu_msg.angular_velocity_covariance = {	pow(0.00085, 2.0f), 0.0, 0.0,
											0.0, pow(0.00085, 2.0f), 0.0, 
											0.0, 0.0, pow(0.00085, 2.0f)}; // fixed
	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance = {	pow(0.00122*9.81, 2.0f), 0.0, 0.0,
												0.0, pow(0.00122*9.81, 2.0f), 0.0,
												0.0, 0.0, pow(0.00122*9.81, 2.0f)}; // fixed

	// NavSatFix sensor_msgs/NavSatFix 
	sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec = 0;
	nav_sat_fix_msg.header.stamp.nanosec = 0;
	nav_sat_fix_msg.header.frame_id = "gps";
	nav_sat_fix_msg.status.status = 0;
	nav_sat_fix_msg.status.service = 1; // fixed to GPS
	nav_sat_fix_msg.latitude = 0.0;
	nav_sat_fix_msg.longitude = 0.0;
	nav_sat_fix_msg.altitude = 0.0;
	nav_sat_fix_msg.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type = 2; // fixed to variance on the diagonal
	
	// DiagnosticsStatus messages for System Status
	diagnostic_msgs::msg::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";

	// DiagnosticsStatus messages for Filter Status
	diagnostic_msgs::msg::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";

  	// Intialising for the log files
	rawtime = time(NULL);
	timeinfo = localtime(&rawtime);
	// sprintf(filename, "Log_%02d-%02d-%02d_%02d-%02d-%02d.anpp", timeinfo->tm_year-100, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	// log_file = fopen(filename, "wb");
  
  	// Initialise packets
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;

	if(state == 0){
		if (OpenComport(args.serdevice, args.baud)){
			printf("Could not open serial port\n");
			exit(EXIT_FAILURE);			
		}
		error = ntrip_initialise(&args, buf);
		if(error){
			printf("ERROR\n");
		}
		else{
			//printf("NOT ERROR\n");
			error += 0;
		}
	}
	if(state == 1 || state == 3){
		if (OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
		{
			printf("Could not open serial port: %s \n",com_port.c_str());
			exit(EXIT_FAILURE);
		}
	}

  	// Request Config packets and also start decoding anpp packets
	SendBuf((unsigned char*)request_all_configuration, sizeof(request_all_configuration));
	an_decoder_initialise(&an_decoder);
  
	while(rclcpp::ok() && !error){
		std::stringstream ss;
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// fwrite(an_decoder_pointer(&an_decoder), sizeof(uint8_t), bytes_received, log_file);
			// Increment the decode buffer length by the number of bytes received 
			an_decoder_increment(&an_decoder, bytes_received);

			// Decode all the packets in the buffer 
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{	
				// System State Packet Decoding
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// NAVSATFIX
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nanosec=system_state_packet.microseconds*1000;

						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							nav_sat_fix_msg.status.status=0;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							nav_sat_fix_msg.status.status=1;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							nav_sat_fix_msg.status.status=2;
						}
						else
						{
							nav_sat_fix_msg.status.status=-1;
						}
						
						nav_sat_fix_msg.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude = system_state_packet.height;

						nav_sat_fix_msg.position_covariance =
							{	pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
								0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
								0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};						

						// IMU
						imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nanosec=system_state_packet.microseconds*1000;

						// Using the RPY orientation as done by cosama
						if(!USE_FLU)
						{
							orientation.setRPY(
								system_state_packet.orientation[0],
								system_state_packet.orientation[1],
								system_state_packet.orientation[2] + MAG_DEC
							);
						}
						else 
						{
							float yaw = -system_state_packet.orientation[2] + M_PI_2 + MAG_DEC;

							if(yaw > M_PI) {
								yaw -= M_PI;
							}

							orientation.setRPY(
								system_state_packet.orientation[0],
								-system_state_packet.orientation[1],
								yaw
							);
						}

        				// // double imuRoll, imuPitch, imuYaw;
						// tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
						// std::cout << "IMU roll pitch yaw: " << std::endl;
						// std::cout << "roll: " << imuRoll*180.0f/M_PI << "\npitch: " << imuPitch*180.0f/M_PI << "\nyaw: " << imuYaw*180.0f/M_PI << std::endl << std::endl;

						if(!USE_FLU)
						{
							imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0]; 
							imu_msg.angular_velocity.y = system_state_packet.angular_velocity[1];
							imu_msg.angular_velocity.z = system_state_packet.angular_velocity[2];
						}
						else
						{
							imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0];
							imu_msg.angular_velocity.y = -system_state_packet.angular_velocity[1];
							imu_msg.angular_velocity.z = -system_state_packet.angular_velocity[2];
						}

						if(!USE_FLU)
						{
							imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0];
							imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[1];
							imu_msg.linear_acceleration.z = system_state_packet.body_acceleration[2];
						}
						else
						{
							imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0];
							imu_msg.linear_acceleration.y = -system_state_packet.body_acceleration[1];
							imu_msg.linear_acceleration.z = -system_state_packet.body_acceleration[2];
						}

						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];
					}
				}
				// Ensure that you free the an_packet when your done with it or you will leak memory                                  
				an_packet_free(&an_packet);

				// PUBLISH MESSAGES
				nav_sat_fix_pub->publish(nav_sat_fix_msg);
				imu_pub->publish(imu_msg);
			}
			
			// Write the logs to the logger reset when counter is full
			// if(write_counter++ >= 100){
			// 	fflush(log_file);
			// 	write_counter = 0;
			// }
		}
		
		if(state == 0){
			error = ntrip(&args, buf, &numbytes);
			remain = numbytes;
			
			// Send Buffer in 255 Byte chunks to the Spatial 
			// Loop till the entire rtcm corrections message is encoded. 
			while(remain)
			{	
				int toCpy = remain > AN_MAXIMUM_PACKET_SIZE ? AN_MAXIMUM_PACKET_SIZE : remain;
				an_packet = encode_rtcm_corrections_packet(toCpy, buf+pos);				
				an_packet_encode(an_packet);
				SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
				an_packet_free(&an_packet);
				pos += toCpy;			

				// Increment buffer
				remain -= toCpy;
			}			
			pos=0;
		}	
		
		rclcpp::spin_some(node);
		//loop_rate.sleep();
	} 
  return 0;
}
