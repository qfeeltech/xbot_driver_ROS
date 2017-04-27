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


#ifndef __QFEEL_TYPES_H
#define __QFEEL_TYPES_H

#include <stdint.h>

#define DEBUG_PRINT 1

#define QBOT_LINK_MAX_PAYLOAD_LEN   255

typedef struct __qbot_message {
	uint8_t id;
	uint8_t len;
	uint8_t payload64[(QBOT_LINK_MAX_PAYLOAD_LEN)/8];
} qbot_message_t;


/***********************************************/
typedef struct __qbot_power {
	uint16_t voltage;
} qbot_power_t;


typedef struct __qbot_infrared {
	uint16_t back_left;
	uint16_t back_central;
	uint16_t back_right;
} qbot_infrared_t;

typedef struct __qbot_ivalue{
	uint16_t front_left;
	uint16_t front_right;
	uint16_t back_left;
	uint16_t back_right;
} qbot_ivalue_t;

typedef struct __qbot_ultrasound {
	uint16_t front_left;
	uint16_t front_central;
	uint16_t front_right;
} qbot_ultrasound_t;

typedef struct __qbot_encoder {
	uint16_t front_left;
	uint16_t front_right;
	uint16_t back_left;
	uint16_t back_right;
} qbot_encoder_t;

typedef struct __qbot_imu {

	int16_t acce_x;
	int16_t acce_y;
	int16_t acce_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;

	float tempter;

	uint32_t time_cnt;

	float yaw;
	float pitch;
	float roll;	
} qbot_imu_t;

typedef struct __qbot_charger {
	char status;
} qbot_charger_t;


#endif	/* __QBOT_TYPES_H */

