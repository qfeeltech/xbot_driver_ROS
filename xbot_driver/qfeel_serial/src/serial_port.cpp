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


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"


// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
Serial_Port::
Serial_Port(char *&uart_name_ , int &baudrate_)
{
	initialize_defaults();
	uart_name = uart_name_;
	baudrate  = baudrate_;
}

Serial_Port::
Serial_Port()
{
	initialize_defaults();
}

Serial_Port::
~Serial_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void
Serial_Port::
initialize_defaults()
{
	// Initialize attributes
	debug  = false;
	fd     = -1;
	status = SERIAL_PORT_CLOSED;

	uart_name = (char*)"/dev/ttyUSB0";
	baudrate  = 57600;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------

int
Serial_Port::
read_message(uint8_t &message)
{
	uint8_t          cp;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	int result = _read_port(cp);


	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if (result > 0)
	{
		message = cp;
//		printf("read-->  %x\n", message);
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
	}

//	printf("read message (serial_port)\n");
	// Done!
	return result;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
Serial_Port::
write_message(qbot_message_t &message)
{
	char buf[128];
	
	// Translate message to buffer
	unsigned len = qbot_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	#if !DEBUG
	_write_port(buf,len);
	#endif

	return len;
}

// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
Serial_Port::
open_serial()
{

	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	printf("OPEN PORT\n");

	fd = _open_port(uart_name);

	// Check success and wait for linking
	while(fd == -1) {
		printf("failure, could not open port.\n----- Wait for linking  1000ms .....\n");
		sleep(1);
		fd = _open_port(uart_name);
	}

	/*
	// Check success
	if (fd == -1)
	{
		printf("failure, could not open port.\n");
		throw EXIT_FAILURE;
		return;
	}
	*/

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	bool success = _setup_port(baudrate, 8, 1, false, false);

	// --------------------------------------------------------------------------
	//   CHECK STATUS
	// --------------------------------------------------------------------------
	if (!success)
	{
		printf("failure, could not configure port.\n");
		throw EXIT_FAILURE;
	}
	if (fd <= 0)
	{
		printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		throw EXIT_FAILURE;
	}

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);

	status = true;

	printf("\n");

	return;

}


uint16_t
Serial_Port::
qbot_msg_to_send_buffer(uint8_t* buffer,
			const qbot_message_t *msg ){
	uint8_t i = 0, check_sum = 0;
	buffer[0] = 0xAA;
	buffer[1] = 0x55;
	buffer[2] = ( (uint8_t)msg->len);
	buffer[3] = msg->id;
	check_sum ^= buffer[2];
	check_sum ^= buffer[3];
	while (i < msg->len-1) {
		buffer[4 + i] = msg->payload64[i];
		check_sum ^= buffer[4 + i];
		i++;
	}
	buffer[i + 4] = check_sum;
	return buffer[2] + 4;
}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void
Serial_Port::
close_serial()
{
	int result = close(fd);

	if ( result )
	{
		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	}

	status = false;
	printf("CLOSE PORT\n");
	printf("\n");	
}


// ------------------------------------------------------------------------------
//   Convenience Functions
// ------------------------------------------------------------------------------
void
Serial_Port::
start()
{
	open_serial();
}

void
Serial_Port::
stop()
{
	close_serial();
}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Serial_Port::
handle_quit( int sig )
{
	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop serial port\n");
	}
	return;
}


// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int
Serial_Port::
_open_port(const char* port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}

	// Finalize
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

	// Done!
	return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool
Serial_Port::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	// Check file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}



// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int
Serial_Port::
_read_port(uint8_t &cp)
{

	// Lock
	pthread_mutex_lock(&lock);
//	printf("_read_port pthread_mutex_lock (serial_port)\n");
	int result = read(fd, &cp, 1);
//	printf("_read_port read (serial_port)\n");
	// Unlock
	pthread_mutex_unlock(&lock);
//	printf("_read_port pthread_mutex_unlock (serial_port)\n");
	return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
void
Serial_Port::
_write_port(char *buf, unsigned &len)
{

	// Lock
//	pthread_mutex_lock(&lock);

	// Write packet via serial link
	write(fd, buf, len);
//	printf("--->%d\n", aa);
	// Wait until all data has been written
	tcdrain(fd);

	// Unlock
//	pthread_mutex_unlock(&lock);
//	printf("done\n");

	return;
}

int16_t
Serial_Port::
uint8_to_int16(uint8_t buf, uint8_t _buf) {
	int16_t tmp;
	tmp = (uint16_t) (buf) | (((uint16_t) (_buf)) << 8);
	return tmp;
}

uint16_t
Serial_Port::
uint8_to_uint16(uint8_t buf, uint8_t _buf) {
	uint16_t tmp;
	tmp = (uint16_t) (buf) + (((uint16_t) (_buf)) << 8);
	return tmp;
}
