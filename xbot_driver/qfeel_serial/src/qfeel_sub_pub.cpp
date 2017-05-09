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


/**
 * @file Qbot_comm.cpp
 *
 * @brief Qbot interface functions
 *
 * Functions for publish and subscribe message from Qbot
 *
 *
 */

#include "qfeel_sub_pub.h"
#include "time.h"

#define ABS(x) (x>0 ? x:(-x))
double EvenFactorial[] = {2.0, 24.0, 720.0, 40320.0, 3628800.0};

uint64_t
get_time_usec() {
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

Qbot_Communication::Qbot_Communication() :
        state_led_pre(0), liner_laser_pre(0),
        motor_state_pre(0), pulse_led_pre(NULL),
        pulse_led_count(0), state_led_count(0), liner_laser_count(0),
        motor_state_count(0), encoder_left_pre(0), encoder_right_pre(0),
        odometry_x(0), odometry_y(0), odometry_th(0),
        odometry_x_pre(0), odometry_y_pre(0), odometry_th_pre(-400), thread_live_flag(1),
//		tfpub_lock(new boost::shared_mutex()),
        pose_init_(false) {
    odometry_info.pose.pose.position.x = 0;
    odometry_info.pose.pose.position.y = 0;
    odometry_info.pose.pose.orientation.w = 1;
    odometry_info.pose.pose.orientation.z = 0;

    // start ros publisher
    ultrasound_pub = comm_nh.advertise<std_msgs::UInt16MultiArray>("/qfeel/ultrasound", 10);
    infrared_pub = comm_nh.advertise<std_msgs::UInt16MultiArray>("/qfeel/infrared", 10);
    imu_pub = comm_nh.advertise<sensor_msgs::Imu>("/qfeel/imu", 10);
    power_pub = comm_nh.advertise<std_msgs::Byte>("/qfeel/power", 10);

    encoder_pub = comm_nh.advertise<std_msgs::UInt16MultiArray>("/qfeel/encoder", 10);
    charger_pub = comm_nh.advertise<std_msgs::Byte>("/qfeel/charger", 10);

//	odometry_pub	= comm_nh.advertise<nav_msgs::Odometry>("odometry", 10);
    odometry_pub = comm_nh.advertise<nav_msgs::Odometry>("/qfeel/odom", 10);
    base_ctr_sub = comm_nh.subscribe("/qfeel/cmd_vel", 1, &Qbot_Communication::base_ctr_Callback, this);
    // test

    //ctrlaw_ctr_sub	= comm_nh.subscribe("/qfeel_follow/control_law",1,&Qbot_Communication::ctrlaw_ctr_Callback,this);

    /* for odometry */
    odometry_info.child_frame_id = comm_nh.param<string>("base_frame", "base_link");
    odometry_info.header.frame_id = comm_nh.param<string>("odom_frame", "odom");
    model_type_ = comm_nh.param<string>("model", "diff");
    last_time = ros::Time::now();
    pthread_create(&tf_pub_tid, NULL, &start_tf_pub_thread, this);
    //pthread_join(tf_pub_tid,NULL);
}


Qbot_Communication::~Qbot_Communication() {
    thread_live_flag = 0;

    pthread_join(tf_pub_tid, NULL);
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Qbot_Communication::write_message(qbot_message_t &message) {
    // do the write
    int len = serial_port->write_message(message);
    // Done!
    return len;
}

/*************** advertise publish ******************/

void
Qbot_Communication::
publish_power(qbot_power_t power) {
    std_msgs::Byte msg;
    double _power;
    //printf("--------------------------> %d\n", power.voltage);
    _power = 0.586397 * (1.0 * power.voltage / 100.0) + 0.000440;
    msg.data = (int)(_power) % 128;
#if DEBUG_PRINT
    printf("-publish power: %f\n", _power);
#endif
    power_pub.publish(msg);

}

void
Qbot_Communication::
publish_infrared(qbot_infrared_t infrared) {
    std_msgs::UInt16MultiArray infrared_msg;
    infrared_msg.data.push_back(infrared.back_left);
    infrared_msg.data.push_back(infrared.back_central);
    infrared_msg.data.push_back(infrared.back_right);
    std::stringstream ss;
    ss << (uint16_t) infrared.back_left << " " << (uint16_t) infrared.back_central << " "
       << (uint16_t) infrared.back_right;
#if DEBUG_PRINT
    std::cout << "-publish infrared: " << ss.str() << endl;
#endif
    infrared_pub.publish(infrared_msg);
    ss.str("");
}

void
Qbot_Communication::
publish_encoder(qbot_encoder_t encoder) {

    std::stringstream ss;
    std_msgs::UInt16MultiArray encoder_msg;
    encoder_msg.data.reserve(4);
    encoder_msg.data.push_back(encoder.front_left);
    encoder_msg.data.push_back(encoder.front_right);
    encoder_msg.data.push_back(encoder.back_left);
    encoder_msg.data.push_back(encoder.back_right);

    ss << (int) encoder.front_left << " " << (int) encoder.front_right << " " << (int) encoder.back_left << " "
       << (int) encoder.back_right;

#if DEBUG_PRINT
    std::cout << "-publish encoder: " << ss.str() << endl;
#endif
    encoder_pub.publish(encoder_msg);

    ss.str("");

    // cal odometry
    publish_odometry(encoder);
}

void
Qbot_Communication::
publish_odometry(qbot_encoder_t encoder) {
    if(model_type_ == "diff"){
        // swap<uint16_t>(encoder.back_left,encoder.back_right);


        if (odometry_th_pre == -400) {    // -400 --> beginning
            odometry_th_pre = 0;
            encoder_left_pre = encoder.back_left;
            encoder_right_pre = encoder.back_right;
        }
        double left_encoder = encoder.back_left * 1.0 - encoder_left_pre * 1.0;
        double right_encoder = encoder.back_right * 1.0 - encoder_right_pre * 1.0;

        if (left_encoder > ENCODER_FULL_HALF) {
            left_encoder -= ENCODER_FULL;
        } else if (left_encoder < (-1 * ENCODER_FULL_HALF)) {
            left_encoder += ENCODER_FULL;
        }

        if (right_encoder > ENCODER_FULL_HALF) {
            right_encoder -= ENCODER_FULL;
        } else if (right_encoder < (-1 * ENCODER_FULL_HALF)) {
            right_encoder += ENCODER_FULL;
        }
        //cout << "encoder: " << left_encoder << " " << right_encoder << endl;
        encoder_left_pre = encoder.back_left;
        encoder_right_pre = encoder.back_right;

        double left_rotary_radians = left_encoder / ENCODER_MAX * 2.0 * PI;
        double right_rotary_radians = right_encoder / ENCODER_MAX * 2.0 * PI;

        double left_travel_dis = left_rotary_radians * WHEEL_R;
        double right_travel_dis = right_rotary_radians * WHEEL_R;
        double body_travel_dis = (left_travel_dis + right_travel_dis) / 2.0;

        double delta_th = (left_travel_dis - right_travel_dis) / WHEEL_D_WIDTH;
        //double rotation_radius = body_travel_dis / delta_th;

        double delta_rate2 = 2.0 * body_travel_dis * body_travel_dis
                             * (1.0 / EvenFactorial[0]
                                - pow(delta_th, 2.0) / EvenFactorial[1]
                                - pow(delta_th, 4.0) / EvenFactorial[2]
                                - pow(delta_th, 6.0) / EvenFactorial[3]);

        double delta_rate = pow(delta_rate2, 1.0 / 2.0) * (body_travel_dis >= 0 ? 1 : (-1));

        double delta_x = delta_rate * cos(odometry_th_pre + delta_th / 2.0);
        double delta_y = delta_rate * sin(odometry_th_pre + delta_th / 2.0);
        //cout << delta_x << " " << delta_y << " " << delta_th << endl;

        odometry_x = odometry_x_pre + delta_x;
        odometry_y = odometry_y_pre + delta_y;
        odometry_th = odometry_th_pre + delta_th;

        // odometry_th = fmod(odometry_th, (2 * PI));

        odometry_x_pre = odometry_x;
        odometry_y_pre = odometry_y;
        odometry_th_pre = odometry_th;
        cout << "Odometry: " << odometry_x_pre << " " << odometry_y_pre << " " << odometry_th_pre << endl;


    }
    else if(model_type_ == "omni"){
        static qbot_encoder_t last_encoder = encoder;
        static bool first_ = true;
        if(first_){
            first_ = false;
        } else{
            double d[5];
            d[1] = encoder.front_right - last_encoder.front_right;
            d[2] = encoder.front_left - last_encoder.front_left;
            d[3] = encoder.back_left - last_encoder.back_left;
            d[4] = encoder.back_right - last_encoder.back_right;
            cout << "d: " << d[1] << " " << d[2] << " " << d[3] << " " << d[4] << endl;

            d[2] *= -1;
            d[3] *= -1;

            for(int i = 1; i <= 4; i++){

                if (d[i] > ENCODER_FULL_HALF) {
                    d[i] -= (1 << 16);
                } else if (d[i] < (-1 * ENCODER_FULL_HALF)) {
                    d[i] += (1 << 16);
                }

                d[i] = d[i] / ENCODER_MAX * 2 * M_PI * WHEEL_R;
            }
            double v_l[3]; // velocity in local coord.
            v_l[0] = (d[1] + d[4] - d[2] - d[3]) * sqrt(2.0) / 2 / 2;
            v_l[1] = (d[1] + d[2] - d[3] - d[4]) * sqrt(2.0) / 2 / 2;
            v_l[2] = (d[1] + d[2] + d[3] + d[4]) / ( WHEEL_D_WIDTH /2 ) / 4 ;
            odometry_x += v_l[0] * cos(odometry_th) - v_l[1] * sin(odometry_th);
            odometry_y += v_l[0] * sin(odometry_th) + v_l[1] * cos(odometry_th);
            odometry_th += v_l[2];
            if(odometry_th > 2*M_PI)
                odometry_th -= 2 * M_PI;
            else if(odometry_th < 0)
                odometry_th += 2 * M_PI;

            cout << "Odometry: " << odometry_x << " " << odometry_y << " " << odometry_th << endl;
            last_encoder = encoder;

        }

    }




/***********************************************************************
  *
  * trans odometry to tf
  *
  * Add 2016.03.25-19:00:00 
  *
  ***/
    curr_time = ros::Time::now();
//	tfpub_lock->lock();
    odometry_info.pose.pose.position.x = odometry_x;
    odometry_info.pose.pose.position.y = odometry_y;
    odometry_info.pose.pose.position.z = 0;
    odometry_info.pose.pose.orientation.w = cos(0.5 * odometry_th);
    odometry_info.pose.pose.orientation.z = sin(0.5 * odometry_th);
//	tfpub_lock->unlock();
    curr_angle = odometry_th;
    odometry_info.header.stamp = curr_time;
    //  ++odometry_info.header.seq;

    if (pose_init_ == false) {
        last_pose.x = odometry_info.pose.pose.position.x;
        last_pose.y = odometry_info.pose.pose.position.y;
        last_angle = curr_angle;
        pose_init_ = true;
    }
    duration = curr_time - last_time;

    odometry_info.twist.twist.linear.x =
            (odometry_info.pose.pose.position.x - last_pose.x) / (duration.sec + 0.000000001 * duration.nsec);
    odometry_info.twist.twist.linear.y =
            (odometry_info.pose.pose.position.y - last_pose.y) / (duration.sec + 0.000000001 * duration.nsec);
    odometry_info.twist.twist.angular.z = (curr_angle - last_angle) / (duration.sec + 0.000000001 * duration.nsec);

    odometry_pub.publish(odometry_info);

    last_angle = curr_angle;
    last_pose.x = odometry_info.pose.pose.position.x;
    last_pose.y = odometry_info.pose.pose.position.y;
    last_time = curr_time;

}

void
Qbot_Communication::
publish_ultrasound(qbot_ultrasound_t ultrasound) {

    std_msgs::UInt16MultiArray ultrasound_msg;
    ultrasound_msg.data.push_back(ultrasound.front_left);
    ultrasound_msg.data.push_back(ultrasound.front_central);
    ultrasound_msg.data.push_back(ultrasound.front_right);

    std::stringstream ss;
    double time_cap = 0.0;
//	测试距离=(高电平时间*声速(340M/S))/2;
    time_cap = 1.0 * ultrasound.front_left * 10.0 / 1000000.0;    // 10us -> s
    ultrasound.front_left = (int) (time_cap * 340.0 * 1000.0 / 2.0);    // m -> mm
    time_cap = 1.0 * ultrasound.front_central * 10.0 / 1000000.0;    // 10us -> s
    ultrasound.front_central = (int) (time_cap * 340.0 * 1000.0 / 2.0);    // m -> mm
    time_cap = 1.0 * ultrasound.front_right * 10.0 / 1000000.0;    // 10us -> s
    ultrasound.front_right = (int) (time_cap * 340.0 * 1000.0 / 2.0);    // m -> mm
    ss << (int) ultrasound.front_left << " " << (int) ultrasound.front_central << " " << (int) ultrasound.front_right;

#if DEBUG_PRINT
    std::cout << "-publish ultrasound: " << ss.str() << "\n";
#endif
    ultrasound_pub.publish(ultrasound_msg);
    ss.str("");
}

void
Qbot_Communication::
publish_imu(qbot_imu_t imu) {
    sensor_msgs::Imu msg;
    float q0, q1, q2, q3, yaw, pitch, roll;

    yaw = imu.yaw;
    pitch = imu.pitch;
    roll = imu.roll;

    q0 = cos(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) - sin(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  //w
    q1 = cos(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw) -
         sin(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw);  //x   pitch
    q2 = sin(0.5 * roll) * cos(0.5 * pitch) * cos(0.5 * yaw) +
         cos(0.5 * roll) * sin(0.5 * pitch) * sin(0.5 * yaw);  //y   roll
    q3 = cos(0.5 * roll) * cos(0.5 * pitch) * sin(0.5 * yaw) +
         sin(0.5 * roll) * sin(0.5 * pitch) * cos(0.5 * yaw);  //z   Yaw

    msg.orientation.w = q0;
    msg.orientation.x = q1;
    msg.orientation.y = q2;
    msg.orientation.z = q3;

    imu_pub.publish(msg);
#if DEBUG_PRINT
    printf("-publish imu: %f %f %f %f\n", q0, q1, q2, q3);
#endif
}

void
Qbot_Communication::
publish_charger(qbot_charger_t charger) {
    std_msgs::Byte msg;
    msg.data = charger.status;
#if DEBUG_PRINT
    printf("-publish charger: %d\n\n", msg.data);
#endif
    charger_pub.publish(msg);
}

/*************** subscribe CallBack ******************/

void
Qbot_Communication::
base_ctr_Callback(const geometry_msgs::Twist::ConstPtr &cmd_vel) {
    unsigned char byte[4];


    float linear_tmp, angular_tmp, y_tmp;
    float linear_x, linear_y, angular_z;
    qbot_message_t message;
    message.len = 0x0D;
    message.id = 0x03;
    linear_tmp = cmd_vel->linear.x;
    if(model_type_ == "omni")
        angular_tmp = -cmd_vel->angular.z;
    else if(model_type_ == "diff")
        angular_tmp = cmd_vel->angular.z;
//	std::cout<< "x: " <<cmd_vel->linear.x<< "  y: " << cmd_vel->linear.y<< "   z: " <<cmd_vel->angular.z<< "\n";
    if (isnan(cmd_vel->linear.x)) {
        linear_x = 0;
    } else linear_x = cmd_vel->linear.x;

    if (isnan(cmd_vel->linear.y)) {
        linear_y = 0;
    } else linear_y = cmd_vel->linear.y;

    if (isnan(cmd_vel->angular.z)) {
        angular_z = 0;
    } else {
        if(model_type_ == "omni")
            angular_z = -cmd_vel->angular.z;
        else if(model_type_ == "diff")
            angular_z = cmd_vel->angular.z;
    }
    linear_pre = linear_tmp;

    angular_pre = angular_tmp;

    memcpy(message.payload64, &linear_x, sizeof(float));
    memcpy(message.payload64 + 4, &linear_y, sizeof(float));
    memcpy(message.payload64 + 8, &angular_z, sizeof(float));

    write_message(message);
}

void
Qbot_Communication::
tf_pub_thread() {
    ros::Rate loooop_rate(100);
    while (thread_live_flag) {
        odom_broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, odometry_info.pose.pose.orientation.z,
                                                     odometry_info.pose.pose.orientation.w),
                                      tf::Vector3(odometry_info.pose.pose.position.x,
                                                  odometry_info.pose.pose.position.y, 0)
                        ),
                        curr_time,
                        odometry_info.header.frame_id,
                        odometry_info.child_frame_id

                )
        );

        loooop_rate.sleep();
    }
    ros::shutdown();
}

void *
start_tf_pub_thread(void *args) {
    // takes an autopilot object argument
    Qbot_Communication *qbot_Communication = (Qbot_Communication *) args;

    // run the object's tf_pub_thread
    qbot_Communication->tf_pub_thread();

    pthread_exit(0);

    // done!
    return NULL;
}

