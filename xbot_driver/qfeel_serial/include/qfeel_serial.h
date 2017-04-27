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


#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/TwistStamped.h>

#include "serial_port.h"
#include "qfeel_pack.h"

int main(int argc, char **argv);
void commands(Qbot_Interface &api);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);


// quit handler
Qbot_Interface *qbot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

