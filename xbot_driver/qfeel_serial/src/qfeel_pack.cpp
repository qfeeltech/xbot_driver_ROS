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


#define __need_timeval
// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "qfeel_pack.h"

int count_write = 0;
timeval t_start, t_end;
long last_time = 0;
long count = 0, total_time = 0;
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------

Qbot_Interface::
Qbot_Interface(Serial_Port *serial_port_) {
    // initialize attributes
    receive_counter = 0;
    qbot_comm.write_count = 0;

    read_time_out = 0;

    reading_status = 0;      // whether the read thread is running
    qbot_comm.writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit = false;  // flag to signal thread exit

    read_tid = 0; // read thread id
    write_tid = 0; // write thread id

    qbot_comm.serial_port = serial_port_; // serial port management object

    RC_Flag = 0;
    rx_wr_index = 0;
    checksum = 0;
    memset(rx_buffer, RX_BUFFER_SIZE, 0);
}

Qbot_Interface::
~Qbot_Interface() {}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Qbot_Interface::
read_messages() {
    uint8_t message;

    bool success;               // receive success flag
    bool received_all = false;  // receive only one message
    bool header00_flag = false, header01_flag = false, data_flag = false;
    //printf("Qbot_Interface::read_messages()\n");
    // Blocking wait for new data
    while (not received_all and not time_to_exit) {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        success = qbot_comm.serial_port->read_message(message);
        //printf("read ---> %d-%x  -  %x\n", rx_wr_index, rx_wr_index, message);
        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if (success) {
            if (message == 0xAA) {
                header00_flag = true;
                RC_Flag |= b_uart_head;
                rx_buffer[rx_wr_index++] = message;
                //printf("read1 ---> %d-%x  -  %x\n", rx_wr_index-1, rx_wr_index-1, message);

            } else if (message == 0x55) {
                if (RC_Flag & b_uart_head) {
                    header01_flag = true;
                    rx_wr_index = 0;
                    RC_Flag &= ~b_rx_over;
                } else
                    rx_buffer[rx_wr_index++] = message;
                //printf("read2 ---> %d-%x  -  %x\n", rx_wr_index-1, rx_wr_index-1, message);

                RC_Flag &= ~b_uart_head;
            } else {
                rx_buffer[rx_wr_index++] = message;
                //printf("read3 ---> %d-%x  -  %x\n", rx_wr_index-1, rx_wr_index-1, message);
                //printf("done %d\n", rx_buffer[0] + 1);
                RC_Flag &= ~b_uart_head;
                if (rx_wr_index == (rx_buffer[0] + 2)) {
                    RC_Flag ^= b_rx_over;
                    data_flag = true;

#if DEBUG_PRINT
                    printf("----------------------- %d -------------------------\n", receive_counter++);
                    printf("Sum_check\n");
#endif
                    read_time_out = 0;    // read the port time_out conter
                    if (Sum_check()) {
#if DEBUG_PRINT
                        printf("Sum_check OK\n");
#endif
/******************************************************************
 * sensor
 ******************************************************************/
                        if (rx_buffer[0] == 0x28 && rx_buffer[1] == 0x10) {
/******************************power******************************************/
                            if (rx_buffer[2] == 0x01) {
#if DEBUG_PRINT
                                printf("power\n");
#endif
                                // swap<uint8_t >(rx_buffer[3], rx_buffer[4]);
                                memcpy(&current_messages.power.voltage, &rx_buffer[3], 2);
//                                printf("voltage:%x\n", current_messages.power.voltage);
                                qbot_comm.publish_power(current_messages.power);
                            }
                            /******************************InfraRed********************************************/
                            if (rx_buffer[5] == 0x03) {
#if DEBUG_PRINT
                                printf("Infrared\n");
#endif
                                memcpy(&current_messages.infrared.back_left, &rx_buffer[6], 2);
                                memcpy(&current_messages.infrared.back_central, &rx_buffer[8], 2);
                                memcpy(&current_messages.infrared.back_right, &rx_buffer[10], 2);
                                qbot_comm.publish_infrared(current_messages.infrared);
                            }
                            /******************************Ivalue********************************************/
                            if (rx_buffer[12] == 0x05) {
#if DEBUG_PRINT
                                printf("Ivalue\n");
#endif
                                memcpy(&current_messages.ivalue.front_left, &rx_buffer[13], 2);
                                memcpy(&current_messages.ivalue.front_right, &rx_buffer[15], 2);
                                memcpy(&current_messages.ivalue.back_left, &rx_buffer[17], 2);
                                memcpy(&current_messages.ivalue.back_right, &rx_buffer[19], 2);
//							qbot_comm.publish_ivalue(current_messages.ivalue);
                            }
/******************************ultrasound********************************************/
                            if (rx_buffer[23] == 0x03) {
#if DEBUG_PRINT
                                printf("ultrasound\n");
#endif
                                memcpy(&current_messages.ultrasound.front_left, &rx_buffer[24], 2);
                                memcpy(&current_messages.ultrasound.front_central, &rx_buffer[26], 2);
                                memcpy(&current_messages.ultrasound.front_right, &rx_buffer[28], 2);
                                qbot_comm.publish_ultrasound(current_messages.ultrasound);
                            }
/******************************encoder********************************************/
                            if (rx_buffer[30] == 0x05) {
#if DEBUG_PRINT
                                printf("encoder\n");
#endif
                                memcpy(&current_messages.encoder.back_left, &rx_buffer[31], 2);
                                memcpy(&current_messages.encoder.back_right, &rx_buffer[33], 2);
                                memcpy(&current_messages.encoder.front_right, &rx_buffer[35], 2);
                                memcpy(&current_messages.encoder.front_left, &rx_buffer[37], 2);
                                qbot_comm.publish_encoder(current_messages.encoder);
                            }
                        }
/*****************************************************************8
 * IMU
 */
                        if (rx_buffer[0] == 0x21 && rx_buffer[1] == 0x11) {
#if DEBUG_PRINT
                            printf("IMU\n");
#endif
                            memcpy(&current_messages.imu.acce_x, &rx_buffer[2], 2);
                            memcpy(&current_messages.imu.acce_y, &rx_buffer[4], 2);
                            memcpy(&current_messages.imu.acce_z, &rx_buffer[6], 2);

                            memcpy(&current_messages.imu.gyro_x, &rx_buffer[8], 2);
                            memcpy(&current_messages.imu.gyro_y, &rx_buffer[10], 2);
                            memcpy(&current_messages.imu.gyro_z, &rx_buffer[12], 2);

                            memcpy(&current_messages.imu.mag_x, &rx_buffer[14], 2);
                            memcpy(&current_messages.imu.mag_y, &rx_buffer[16], 2);
                            memcpy(&current_messages.imu.mag_z, &rx_buffer[18], 2);

                            memcpy(&current_messages.imu.tempter, &rx_buffer[20], 4);

                            memcpy(&current_messages.imu.time_cnt, &rx_buffer[24], 4);

                            qbot_comm.publish_imu(current_messages.imu);

                            printf("acce: %d %d %d\ngyro: %d%d %d\nmag : %d %d%d\n", \
                                    current_messages.imu.acce_x, current_messages.imu.acce_y, \
                                    current_messages.imu.acce_z, current_messages.imu.gyro_x, \
                                    current_messages.imu.gyro_y, current_messages.imu.gyro_z, \
                                    current_messages.imu.mag_x, current_messages.imu.mag_y, \
                                    current_messages.imu.mag_z);
                        }
                    }
                }
            }
        } // end: if read message

        // Check for receipt of all items
        received_all =
                header00_flag &&
                header01_flag &&
                data_flag;

        // give the write thread time to use the port
        //if ( qbot_comm.writing_status > false )
        //	usleep(100); // look for components of batches at 10kHz

    } // end: while not received all
    return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Qbot_Interface::
write_message(qbot_message_t message) {
    // do the write
    int len = qbot_comm.serial_port->write_message(message);

    // Done!
    return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start() {
    int result;

    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------

    if (not qbot_comm.serial_port->status == 1) // SERIAL_PORT_OPEN
    {
        fprintf(stderr, "ERROR: serial port not open\n");
        throw 1;
    }
    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    printf("START READ THREAD \n");

    result = pthread_create(&read_tid, NULL, &start_qbot_interface_read_thread, this);
    if (result) throw result;

    // now we're reading messages
    printf("\n");


    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    //printf("START WRITE THREAD \n");

    //result = pthread_create( &write_tid, NULL, &start_qbot_interface_write_thread, this );
    //if ( result ) throw result;
    // wait for it to be started

    //while ( not qbot_comm.writing_status )
    //	usleep(10000); // 10Hz

    // now we're streaming setpoint commands
    //printf("\n");

    // Done!

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Qbot_Interface::
stop() {

    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------

    // signal exit
    time_to_exit = true;
    if (!pthread_kill(read_tid, 0)) {
        return;
    }

    // wait for exit
    if (0 == pthread_join(read_tid, NULL)) {
        printf("read pthread close\n");
    } else {
        printf("Error: close read pthread fail\n");
    }

    //if (0 == pthread_join(write_tid,NULL)) {
    //	printf("write pthread close\n");
    //} else {
    //	printf("Error: close write pthread fail\n");
    //}
    // still need to close the serial_port separately
    exit(0);
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start_read_thread() {

    if (reading_status != 0) {
        fprintf(stderr, "read thread already running\n");
        return;
    } else {
        read_thread();
        return;
    }

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
start_write_thread(void) {
    if (not qbot_comm.writing_status == false) {
        fprintf(stderr, "write thread already running\n");
        return;
    } else {
        write_thread();
        return;
    }

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Qbot_Interface::
handle_quit(int sig) {

    try {
        stop();

    }
    catch (int error) {
        fprintf(stderr, "Warning, could not stop autopilot interface\n");
    }

}


// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
read_thread() {
    reading_status = true;

    while (not time_to_exit) {
        read_messages();
        usleep(15000); // Read batches at 40Hz
        read_time_out++;
        if (read_time_out > 100) {
            /*
                Do something
            */
        }
    }

    reading_status = false;

    return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Qbot_Interface::
write_thread(void) {
    // signal startup
    qbot_comm.writing_status = 2;

    // write a message and signal writing
//	write_setpoint();
    qbot_comm.writing_status = true;
    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while (not time_to_exit) {
        usleep(250000);   // Stream at 40Hz
        count_write++;
//		usleep(4000);   // Stream at 25Hz

    }
    // signal end
    qbot_comm.writing_status = false;

    return;

}


uint8_t
Qbot_Interface::
Sum_check() {
    uint8_t i;
    unsigned int checksum = 0;
    for (i = 0; i < rx_buffer[0] + 2; i++)
        checksum ^= rx_buffer[i];

    return checksum ? 0 : 1;
}


// End Qbot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void *
start_qbot_interface_read_thread(void *args) {
    // takes an autopilot object argument
    Qbot_Interface *qbot_Interface = (Qbot_Interface *) args;

    // run the object's read thread
    qbot_Interface->start_read_thread();

    // done!
    return NULL;
}

void *
start_qbot_interface_write_thread(void *args) {
    // takes an autopilot object argument
    Qbot_Interface *qbot_Interface = (Qbot_Interface *) args;

    // run the object's read thread
    qbot_Interface->start_write_thread();

    // done!
    return NULL;
}


