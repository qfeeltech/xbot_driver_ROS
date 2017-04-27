/**
 *   XBOT Driver, the driver of the XBOT robot.
 *   Copyright (C) 2017  QFeeltech.
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef QFEEL_SUB_PUB_H_
#define QFEEL_SUB_PUB_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <iostream>
#include <sstream>
#include <math.h>
using namespace std;

#include "pthread.h" // This uses POSIX Threads
#include <boost/thread/shared_mutex.hpp>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "qfeel_serial/imu.h"

#include "serial_port.h"
#include "qfeel_types.h"


#define WHEEL_R	0.085
#define WHEEL_D_WIDTH	0.40		
#define PI	3.1415926535898

#define ENCODER_MAX		(8000)		// circle
#define ENCODER_FULL	(1<<16)		// counter full
#define ENCODER_FULL_HALF	(ENCODER_FULL/2)

// ----------------------------------------------------------------------------------
//   Qbot Interface Class
// ----------------------------------------------------------------------------------
class Qbot_Communication{

public:
	Qbot_Communication();
	~Qbot_Communication();
	
	char writing_status;
	uint64_t write_count;
	ros::NodeHandle comm_nh;

	ros::Publisher power_pub, encoder_pub, odometry_pub, ultrasound_pub, infrared_pub, imu_pub, charger_pub;
	ros::Subscriber base_ctr_sub,cmd_vel_sub,up_down_sub;
		
	int  write_message(qbot_message_t &message);
		
	void tf_pub_thread(void);
	void publish_odometry(qbot_encoder_t encoder);
	void publish_power(qbot_power_t power);
	void publish_encoder(qbot_encoder_t encoder);
	void publish_ultrasound(qbot_ultrasound_t ultrasound);
	void publish_infrared(qbot_infrared_t infrared);
	void publish_imu(qbot_imu_t imu);
	void publish_charger(qbot_charger_t charger);

	void base_ctr_Callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
//	void up_down_Callback(const qfeel_keyop::msg::ConstPtr& location);

	Serial_Port *serial_port;

private:
	
	/* for odometry */	
    bool thread_live_flag;
	nav_msgs::Odometry odometry_info;
	ros::Time last_time, curr_time;
	ros::Duration duration;
	geometry_msgs::Point last_pose; 
	float last_angle, curr_angle;
	bool pose_init_;
	pthread_t tf_pub_tid; 
	tf::TransformBroadcaster odom_broadcaster;
	/****** end *****/

	int encoder_left_pre, encoder_right_pre;
	char* pulse_led_pre, pulse_led_count;
	char state_led_pre, state_led_count;
	char liner_laser_pre, liner_laser_count;
	char motor_state_pre, motor_state_count;
	float linear_pre, angular_pre;
	double odometry_x, odometry_y, odometry_th;
	double odometry_x_pre, odometry_y_pre, odometry_th_pre;
	string model_type_;
};

void* start_tf_pub_thread(void* args);

#endif // QBOT_COMM_H_
