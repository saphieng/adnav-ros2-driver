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
#define MAG_DEC 0.22008602

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
	auto nav_sat_origin_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/origin", 10);
	auto magnetic_field_pub = node->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
	auto barometric_pressure_pub = node->create_publisher<sensor_msgs::msg::FluidPressure>("/imu/baro", 10);
	auto temperature_pub = node->create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);
	auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("/imu/twist", 10);
	auto pose_pub = node->create_publisher<geometry_msgs::msg::Pose>("/imu/pose", 10);
	auto system_status_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/imu/systemStatus", 10);
	auto filter_status_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("imu/filterStatus", 10);
	auto gnss_fix_type_pub = node->create_publisher<std_msgs::msg::String>("/gps/fixType", 10);
	auto nav_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/gps/odom", 10);
	auto nav_path_pub = node->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);

	nav_msgs::msg::Path path_msg;
	int path_pose_count = 1000;

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
	imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = 0.0;
	imu_msg.angular_velocity_covariance = {	pow(0.0014, 2.0f), 0.0, 0.0,
											0.0, pow(0.0014, 2.0f), 0.0, 
											0.0, 0.0, pow(0.0014, 2.0f)}; // fixed
	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance = {	pow(0.002, 2.0f), 0.0, 0.0,
												0.0, pow(0.002, 2.0f), 0.0,
												0.0, 0.0, pow(0.002, 2.0f)}; // fixed

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

	sensor_msgs::msg::NavSatFix nav_sat_origin_msg;
	nav_sat_origin_msg = nav_sat_fix_msg;
	
  	// MagneticField geometry_msg/magnetic_field
	sensor_msgs::msg::MagneticField magnetic_field_msg;
	magnetic_field_msg.magnetic_field.x = 0;
	magnetic_field_msg.magnetic_field.y = 0;
	magnetic_field_msg.magnetic_field.z = 0;
	magnetic_field_msg.header.frame_id = "rawsensors_magnetometer";

	// Barometric Pressure sensor_msgs/fluidPressure
	sensor_msgs::msg::FluidPressure barometric_pressure_msg;
	barometric_pressure_msg.fluid_pressure=0;
	barometric_pressure_msg.header.frame_id = "barometric_pressure";

	// Temperature sensor_msgs/Temperature
	sensor_msgs::msg::Temperature temperature_msg;
	temperature_msg.temperature=0;
	temperature_msg.header.frame_id = "temperature";

	// Twist sensor_msgs/twist
	geometry_msgs::msg::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

  	// Position in ECEF Postion (Packet 33) and Orientation in Quartenion Format (Same as IMU)
	geometry_msgs::msg::Pose pose_msg;
	pose_msg.position.x = 0;
	pose_msg.position.y = 0;
	pose_msg.position.z = 0;
	pose_msg.orientation.x=0.0;
	pose_msg.orientation.y=0.0;
	pose_msg.orientation.z=0.0;
	pose_msg.orientation.w=0.0;

	// Odom_msgs/NavOdom
	double cov[36] = {
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  // Row 1
        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  // Row 2
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  // Row 3
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  // Row 4
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  // Row 5
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1   // Row 6
    };

	nav_msgs::msg::Odometry odom_msg;
	odom_msg.header.stamp.sec = 0;
	odom_msg.header.stamp.nanosec = 0;
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "gps";
	odom_msg.pose.pose = pose_msg;
	std::copy(std::begin(cov), std::end(cov), std::begin(odom_msg.pose.covariance));
	odom_msg.twist.twist = twist_msg;
	std::copy(std::begin(cov), std::end(cov), std::begin(odom_msg.twist.covariance));

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
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	ecef_position_packet_t ecef_position_packet;
	raw_gnss_packet_t raw_gnss_packet;

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
				// Raw GNSS Packet Decoding
				if(an_packet->id == packet_id_raw_gnss)
				{
					if(decode_raw_gnss_packet(&raw_gnss_packet, an_packet) == 0)
					{

					}
				}

				// System State Packet Decoding
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// GNSS FIX TYPE
						switch(system_state_packet.filter_status.b.gnss_fix_type)
						{
							case 0: 
								gnssFixType.str("No GNSS fix");
								break;
							case 1:
								gnssFixType.str("2D Fix");
								break;
							case 2:
								gnssFixType.str("3D Fix");
								break;
							case 3:
								gnssFixType.str("SBAS Fix");
								break;
							case 4:
								gnssFixType.str("Differential Fix");
								break;
							case 5:
								gnssFixType.str("Omnistar/Starfire Fix");
								break;
							case 6:
								gnssFixType.str("RTK Float");
								break;
							case 7:
								gnssFixType.str("RTK Fixed");
								break;
							default:
								gnssFixType.str("NOT CONNECTED");
						}						
						gnss_fix_type_msgs.data = gnssFixType.str();

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

						// TWIST
						if(!USE_FLU)
						{
							twist_msg.linear.x = system_state_packet.velocity[0];
							twist_msg.linear.y = system_state_packet.velocity[1];
							twist_msg.linear.z = system_state_packet.velocity[2];
							twist_msg.angular.x = system_state_packet.angular_velocity[0];
							twist_msg.angular.y = system_state_packet.angular_velocity[1];
							twist_msg.angular.z = system_state_packet.angular_velocity[2];
						}
						else 
						{
							twist_msg.linear.x = system_state_packet.velocity[0];
							twist_msg.linear.y = -system_state_packet.velocity[1];
							twist_msg.linear.z = -system_state_packet.velocity[2];
							twist_msg.angular.x = system_state_packet.angular_velocity[0];
							twist_msg.angular.y = -system_state_packet.angular_velocity[1];
							twist_msg.angular.z = -system_state_packet.angular_velocity[2];
						}

						// NAV ODOM
						odom_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						odom_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;

						// LLA->ENU, better accuacy than gpsTools especially for z value
						double x, y, z;
						Eigen::Vector3d lla(nav_sat_fix_msg.latitude, nav_sat_fix_msg.longitude, nav_sat_fix_msg.altitude);

						if (!initENU) 
						{
							std::cout << "INIT GPS Geo Converter: " << lla(0) << ", " <<  lla(1) << ", " << lla(2) << std::endl; 
							geo_converter.Reset(lla(0), lla(1), lla(2));

							nav_sat_origin_msg.latitude = lla(0);
							nav_sat_origin_msg.longitude = lla(1);
							nav_sat_origin_msg.altitude = lla(2);

							initENU = true;
						}

						geo_converter.Forward(lla(0), lla(1), lla(2), x, y, z);
						Eigen::Vector3d enu(x, y, z);

						// std::cout << "GPS ENU: " << std::endl; 
						// std::cout << "x: " << enu(0) << "\ny: " << enu(1) << "\nz: " << enu(2) << std::end

						if(USE_ENU)
						{
							odom_msg.pose.pose.position.x = enu(0);
							odom_msg.pose.pose.position.y = enu(1);
							odom_msg.pose.pose.position.z = enu(2);
						}
						else // NED
						{
							odom_msg.pose.pose.position.x = enu(1);
							odom_msg.pose.pose.position.y = enu(0);
							odom_msg.pose.pose.position.z = -enu(2);
						}

						tf2::Quaternion headingQuat;
						if(USE_ENU)
						{
							float enuHeading = ((2*M_PI - raw_gnss_packet.heading) + M_PI_2) + MAG_DEC;
							if(enuHeading > 2*M_PI)
							{
								enuHeading -= 2*M_PI;
							}
							headingQuat.setRPY(0, 0, enuHeading);
						}
						else // NED
						{
							headingQuat.setRPY(0, 0, raw_gnss_packet.heading + MAG_DEC);
						}

						odom_msg.pose.pose.orientation.x = headingQuat.getAxis().getX();
						odom_msg.pose.pose.orientation.y = headingQuat.getAxis().getY();
						odom_msg.pose.pose.orientation.z = headingQuat.getAxis().getZ();
						odom_msg.pose.pose.orientation.w = headingQuat.getW();

						odom_msg.pose.covariance[0]  = pow(system_state_packet.standard_deviation[0], 2.0f);
						odom_msg.pose.covariance[7]  = pow(system_state_packet.standard_deviation[1], 2.0f);
						odom_msg.pose.covariance[14] = pow(system_state_packet.standard_deviation[2], 2.0f);

						// NAV PATH
						// publish path
						path_msg.header.frame_id = odom_msg.header.frame_id;
						path_msg.header.stamp = odom_msg.header.stamp;

						geometry_msgs::msg::PoseStamped path_pose;
						path_pose.header = path_msg.header;
						path_pose.pose.position.x = odom_msg.pose.pose.position.x;
						path_pose.pose.position.y = odom_msg.pose.pose.position.y;
						path_pose.pose.position.z = odom_msg.pose.pose.position.z;
						path_pose.pose.orientation.x = odom_msg.pose.pose.orientation.x;
						path_pose.pose.orientation.y = odom_msg.pose.pose.orientation.y;
						path_pose.pose.orientation.z = odom_msg.pose.pose.orientation.z;
						path_pose.pose.orientation.w = odom_msg.pose.pose.orientation.w;

						if(path_msg.poses.size() >= path_pose_count)
						{
							path_msg.poses.erase(path_msg.poses.begin());
						}
						path_msg.poses.push_back(path_pose);

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

						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						// POSE Orientation
						pose_msg.orientation.x = orientation[0];
						pose_msg.orientation.y = orientation[1];
						pose_msg.orientation.z = orientation[2];
						pose_msg.orientation.w = orientation[3];

						// SYSTEM STATUS
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
						}

						// FILTER STATUS - this looks like it doesnt work, as each consecutive condition can overwrite the previous one...
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
						}
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
						}
					}
				}

				// ECEF Position (in meters) Packet for Pose Message
				if(an_packet->id == packet_id_ecef_position)
				{		
					if(decode_ecef_position_packet(&ecef_position_packet, an_packet) == 0)
					{
						pose_msg.position.x = ecef_position_packet.position[0];
						pose_msg.position.y = ecef_position_packet.position[1];
						pose_msg.position.z = ecef_position_packet.position[2];
					}
				}

				// QUATERNION ORIENTATION STANDARD DEVIATION PACKET 
				if (an_packet->id == packet_id_quaternion_orientation_standard_deviation)
				{
					// copy all the binary data into the typedef struct for the packet 
					// this allows easy access to all the different values             
					if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
					{
						imu_msg.orientation_covariance[0] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[0], 2.0f);
						imu_msg.orientation_covariance[4] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[1], 2.0f);
						imu_msg.orientation_covariance[8] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[2], 2.0f);
					
						odom_msg.pose.covariance[21] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[0], 2.0f);
						odom_msg.pose.covariance[28] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[1], 2.0f);
						odom_msg.pose.covariance[35] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[2], 2.0f);
					}
				}

				// Setting up the magnetic field to display
				if(an_packet->id == packet_id_raw_sensors){
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0){
						// Time Stamp from the System State Packet
						magnetic_field_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						magnetic_field_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;

						barometric_pressure_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						barometric_pressure_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;

						temperature_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						temperature_msg.header.stamp.nanosec = system_state_packet.microseconds*1000;

						// RAW MAGNETICFIELD VALUE FROM IMU
						if(!USE_FLU)
						{
							magnetic_field_msg.magnetic_field.x = raw_sensors_packet.magnetometers[0];
							magnetic_field_msg.magnetic_field.y = raw_sensors_packet.magnetometers[1];
							magnetic_field_msg.magnetic_field.z = raw_sensors_packet.magnetometers[2];
						}
						else
						{
							magnetic_field_msg.magnetic_field.x = raw_sensors_packet.magnetometers[0];
							magnetic_field_msg.magnetic_field.y = -raw_sensors_packet.magnetometers[1];
							magnetic_field_msg.magnetic_field.z = -raw_sensors_packet.magnetometers[2];
						}

			
						if(!USE_FLU)
						{
							imu_msg.angular_velocity.x = raw_sensors_packet.gyroscopes[0]; 
							imu_msg.angular_velocity.y = raw_sensors_packet.gyroscopes[1];
							imu_msg.angular_velocity.z = raw_sensors_packet.gyroscopes[2];
						}
						else
						{
							imu_msg.angular_velocity.x = raw_sensors_packet.gyroscopes[0];
							imu_msg.angular_velocity.y = -raw_sensors_packet.gyroscopes[1];
							imu_msg.angular_velocity.z = -raw_sensors_packet.gyroscopes[2];
						}

						rstd_gyr_x.add_value(imu_msg.angular_velocity.x);
						rstd_gyr_y.add_value(imu_msg.angular_velocity.y);
						rstd_gyr_z.add_value(imu_msg.angular_velocity.z);
						
						// std::cout << "STD Gryo: " << (rstd_gyr_x.standard_deviation() + rstd_gyr_y.standard_deviation() + rstd_gyr_z.standard_deviation())/3.0f << std::endl;
						// std::cout << "x: " << rstd_gyr_x.standard_deviation() << "\ny: " << rstd_gyr_y.standard_deviation() << "\nz: " << rstd_gyr_z.standard_deviation() << std::endl << std::endl;

						if(!USE_FLU)
						{
							imu_msg.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
							imu_msg.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
							imu_msg.linear_acceleration.z = raw_sensors_packet.accelerometers[2];
						}
						else
						{
							imu_msg.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
							imu_msg.linear_acceleration.y = -raw_sensors_packet.accelerometers[1];
							imu_msg.linear_acceleration.z = -raw_sensors_packet.accelerometers[2];
						}

						rstd_acc_x.add_value(imu_msg.linear_acceleration.x);
						rstd_acc_y.add_value(imu_msg.linear_acceleration.y);
						rstd_acc_z.add_value(imu_msg.linear_acceleration.z);
						
						// std::cout << "STD Acc: " << (rstd_acc_x.standard_deviation() + rstd_acc_y.standard_deviation() + rstd_acc_z.standard_deviation())/3.0f << std::endl;
						// std::cout << "x: " << rstd_acc_x.standard_deviation() << "\ny: " << rstd_acc_y.standard_deviation() << "\nz: " << rstd_acc_z.standard_deviation() << std::endl << std::endl;

						// BAROMETRIC PRESSURE
						barometric_pressure_msg.fluid_pressure = raw_sensors_packet.pressure;

						// TEMPERATURE
						temperature_msg.temperature = raw_sensors_packet.pressure_temperature;
					}
				}

				// Ensure that you free the an_packet when your done with it or you will leak memory                                  
				an_packet_free(&an_packet);

				// std::cout << "ANG VEL: " << std::endl;
				// std::cout << "x: " << imu_msg.angular_velocity.x << "\ny: " << imu_msg.angular_velocity.y << "\nz: " << imu_msg.angular_velocity.z << std::endl;
				// std::cout << "LIN ACC: " << std::endl;
				// std::cout << "x: " << imu_msg.linear_acceleration.x << "\ny: " << imu_msg.linear_acceleration.y << "\nz: " << imu_msg.linear_acceleration.z << std::endl;
				
				// double imuRoll, imuPitch, imuYaw;
				// tf2::Quaternion orient(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
				// tf2::Matrix3x3(orient).getRPY(imuRoll, imuPitch, imuYaw);
				// std::cout << "RPY: " << std::endl;
				// std::cout << "roll: " << imuRoll*180.0f/M_PI << "\npitch: " << imuPitch*180.0f/M_PI << "\nyaw: " << imuYaw*180.0f/M_PI << std::endl;

				// std::cout << "POSE MSG (ECEF) POS: " << std::endl;
				// std::cout << "x: " << pose_msg.position.x << "\ny: " << pose_msg.position.y << "\nz: " << pose_msg.position.z << std::endl;
				// std::cout << "ODOM MSG POS: " << std::endl;
				// std::cout << "x: " << odom_msg.pose.pose.position.x << "\ny: " << odom_msg.pose.pose.position.y << "\nz: " << odom_msg.pose.pose.position.z << std::endl;

				// std::cout << "GNSS POS: " << std::endl;
				// std::cout << "x: " << raw_gnss_packet.position[0] << "\ny: " << raw_gnss_packet.position[1] << "\nz: " << raw_gnss_packet.position[2] << std::endl;
				// std::cout << "GNSS POS STD: " << std::endl;
				// std::cout << "x: " << raw_gnss_packet.position_standard_deviation[0] << "\ny: " << raw_gnss_packet.position_standard_deviation[1] << "\nz: " << raw_gnss_packet.position_standard_deviation[2] << std::endl;
				// std::cout << "GNSS VEL: " << std::endl;
				// std::cout << "x: " << raw_gnss_packet.velocity[0] << "\ny: " << raw_gnss_packet.velocity[1] << "\nz: " << raw_gnss_packet.velocity[2] << std::endl;
				// std::cout << "GNSS Heading: " << std::endl;
				// std::cout << "Heading: " << raw_gnss_packet.heading*180.0/M_PI  << std::endl;

				// PUBLISH MESSAGES
				nav_sat_fix_pub->publish(nav_sat_fix_msg);
				nav_sat_origin_pub->publish(nav_sat_origin_msg);
				twist_pub->publish(twist_msg);
				imu_pub->publish(imu_msg);
				system_status_pub->publish(system_status_msg);
				filter_status_pub->publish(filter_status_msg);
				magnetic_field_pub->publish(magnetic_field_msg);
				barometric_pressure_pub->publish(barometric_pressure_msg);
				temperature_pub->publish(temperature_msg);
				pose_pub->publish(pose_msg);
				gnss_fix_type_pub->publish(gnss_fix_type_msgs);
				nav_odom_pub->publish(odom_msg);
				nav_path_pub->publish(path_msg);
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
