/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011  Bill Nesbitt
*/

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "serial.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/select.h>
#include <string.h>

serialStruct_t *initSerial(const char *port, unsigned int baud, char ctsRts) {
	serialStruct_t *s;
	struct termios options;
	unsigned int brate;

	s = (serialStruct_t *)calloc(1, sizeof(serialStruct_t));

	s->fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (s->fd == -1) {
		free(s);
		return 0;
	}

	fcntl(s->fd, F_SETFL, 0);    // clear all flags on descriptor, enable direct I/O

//	bzero(&options, sizeof(options));
//	memset(&options, 0, sizeof(options));
	tcgetattr(s->fd, &options);

#ifdef B921600
	switch (baud) {
		case 9600:
			brate = B9600;
			break;
		case 19200:
			brate = B19200;
			break;
		case 38400:
			brate = B38400;
			break;
		case 57600:
			brate = B57600;
			break;
		case 115200:
			brate = B115200;
			break;
		case 230400:
			brate = B230400;
			break;
		case 460800:
			brate = B460800;
			break;
		case 921600:
			brate = B921600;
			break;
		default:
			brate = B115200;
			break;
	}
	options.c_cflag = brate;
#else   // APPLE
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
#endif

	options.c_cflag |= CRTSCTS | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_iflag &= (~(IXON|IXOFF|IXANY));	// turn off software flow control
	options.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	options.c_lflag = 0;

	options.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	options.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

#ifdef CCTS_OFLOW
	options.c_cflag |= CCTS_OFLOW;
#endif

	if (!ctsRts)
		options.c_cflag &= ~(CRTSCTS);	// turn off hardware flow control

	// set the new port options
	tcsetattr(s->fd, TCSANOW, &options);

	return s;
}

void serialFree(serialStruct_t *s) {
	if (s) {
		if (s->fd)
			close(s->fd);
		free (s);
	}
}

void serialNoParity(serialStruct_t *s) {
	struct termios options;

	tcgetattr(s->fd, &options);   // read serial port options

	options.c_cflag &= ~(PARENB | CSTOPB);

	tcsetattr(s->fd, TCSANOW, &options);
}

void serialEvenParity(serialStruct_t *s) {
	struct termios options;

	tcgetattr(s->fd, &options);   // read serial port options

	options.c_cflag |= (PARENB);
	options.c_cflag &= ~(PARODD | CSTOPB);

	tcsetattr(s->fd, TCSANOW, &options);
}

void serialWriteChar(serialStruct_t *s, unsigned char c) {
	char ret;

	ret = write(s->fd, &c, 1);
}

void serialWrite(serialStruct_t *s, const char *str, unsigned int len) {
	char ret;

	ret = write(s->fd, str, len);
}

unsigned char serialAvailable(serialStruct_t *s) {
	fd_set fdSet;
	struct timeval timeout;

	FD_ZERO(&fdSet);
	FD_SET(s->fd, &fdSet);
	memset((char *)&timeout, 0, sizeof(timeout));

	if (select(s->fd+1, &fdSet, 0, 0, &timeout) == 1)
		return 1;
	else
		return 0;
}

void serialFlush(serialStruct_t *s) {
	while (serialAvailable(s))
		serialRead(s);
}

unsigned char serialRead(serialStruct_t *s) {
	char ret;
	unsigned char c;

	ret = read(s->fd, &c, 1);

	return c;
}
