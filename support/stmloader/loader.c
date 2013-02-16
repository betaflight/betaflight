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

#include "serial.h"
#include "stmbootloader.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdlib.h>

#define DEFAULT_PORT		"/dev/ttyUSB0"
#define DEFAULT_BAUD		115200
#define FIRMWARE_FILENAME	"STM32.hex"

serialStruct_t *s;

char port[256];
unsigned int baud;
unsigned char overrideParity;
unsigned char noSendR;
char firmFile[256];

void loaderUsage(void) {
	fprintf(stderr, "usage: loader <-h> <-p device_file> <-b baud_rate> <-f firmware_file> <-o> <-n>\n");
}

unsigned int loaderOptions(int argc, char **argv) {
	int ch;

	strncpy(port, DEFAULT_PORT, sizeof(port));
	baud = DEFAULT_BAUD;
	overrideParity = 0;
	noSendR = 0;
	strncpy(firmFile, FIRMWARE_FILENAME, sizeof(firmFile));

	/* options descriptor */
	static struct option longopts[] = {
		{ "help",					required_argument,	NULL,		'h' },
		{ "port",					required_argument,	NULL,		'p' },
		{ "baud",					required_argument,	NULL,		's' },
		{ "firm_file",				required_argument,	NULL,		'f' },
		{ "override_parity",		no_argument,			NULL,		'o' },
		{ "no_send_r",				no_argument,			NULL,		'n' },
		{ NULL,						0,							NULL,		0 }
	};

	while ((ch = getopt_long(argc, argv, "hp:b:f:o:n", longopts, NULL)) != -1)
		switch (ch) {
		case 'h':
			loaderUsage();
			exit(0);
			break;
		case 'p':
			strncpy(port, optarg, sizeof(port));
			break;
		case 'b':
			baud = atoi(optarg);
			break;
		case 'f':
			strncpy(firmFile, optarg, sizeof(firmFile));
			break;
		case 'o':
			overrideParity = 1;
			break;
		case 'n':
			noSendR = 1;
			break;
		default:
			loaderUsage();
			return 0;
	}
	argc -= optind;
	argv += optind;

	return 1;
}

int main(int argc, char **argv) {
	FILE *fw;

	// init
	if (!loaderOptions(argc, argv)) {
		fprintf(stderr, "Init failed, aborting\n");
		return 1;
	}

	s = initSerial(port, baud, 0);
	if (!s) {
		fprintf(stderr, "Cannot open serial port '%s', aborting.\n", port);
		return 1;
	}

	fw = fopen(firmFile, "r");
	if (!fw) {
		fprintf(stderr, "Cannot open firmware file '%s', aborting.\n", firmFile);
		return 1;
	}
	else {
		printf("Upgrading STM on port %s from %s...\n", port, firmFile);
		stmLoader(s, fw, overrideParity, noSendR);
	}

	return 0;
}
