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
#include <string.h>
#include <ctype.h>

#define STM_RETRIES_SHORT	1000
#define STM_RETRIES_LONG	50000

unsigned char getResults[11];

unsigned char stmHexToChar(const char *hex) {
	char hex1, hex2;
	unsigned char nibble1, nibble2;

	// force data to upper case
	hex1 = toupper(hex[0]);
	hex2 = toupper(hex[1]);

	if (hex1 < 65)
		nibble1 = hex1 - 48;
	else
		nibble1 = hex1 - 55;

	if (hex2 < 65)
		nibble2 = hex2 - 48;
	else
		nibble2 = hex2 - 55;

	return (nibble1 << 4 | nibble2);
}


unsigned char stmWaitAck(serialStruct_t *s, int retries) {
	unsigned char c;
	unsigned int i;

	for (i = 0; i < retries; i++) {
		if (serialAvailable(s)) {
			c = serialRead(s);
			if (c == 0x79) {
//				putchar('+'); fflush(stdout);
				return 1;
			}
			if (c == 0x1f) {
				putchar('-'); fflush(stdout);
				return 0;
			}
			else {
				printf("?%x?", c); fflush(stdout);
				return 0;
			}
		}
		usleep(500);
	}

	return 0;
}

unsigned char stmWrite(serialStruct_t *s, const char *hex) {
	unsigned char c;
	unsigned char ck;
	unsigned char i;

	ck = 0;
	i = 0;
	while (*hex) {
		c = stmHexToChar(hex);
		serialWrite(s, (char *)&c, 1);
		ck ^= c;
		hex += 2;
		i++;
	}
	if (i == 1)
		ck = 0xff ^ c;

	// send checksum
	serialWrite(s, (char *)&ck, 1);

	return stmWaitAck(s, STM_RETRIES_LONG);
}

void stmWriteCommand(serialStruct_t *s, char *msb, char *lsb, char *len, char *data) {
	char startAddress[9];
	char lenPlusData[128];
	char c;

	strncpy(startAddress, msb, sizeof(startAddress));
	strcat(startAddress, lsb);

	sprintf(lenPlusData, "%02x%s", stmHexToChar(len) - 1, data);

	write:
	// send WRITE MEMORY command
	do {
		c = getResults[5];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	// send address
	if (!stmWrite(s, startAddress)) {
		putchar('A');
		goto write;
	}

	// send len + data
	if (!stmWrite(s, lenPlusData)) {
		putchar('D');
		goto write;
	}

	putchar('='); fflush(stdout);
}

char *stmHexLoader(serialStruct_t *s, FILE *fp) {
	char hexByteCount[3], hexAddressLSB[5], hexRecordType[3], hexData[128];
	char addressMSB[5];
	static char addressJump[9];

//	bzero(addressJump, sizeof(addressJump));
//	bzero(addressMSB, sizeof(addressMSB));
	memset(addressJump, 0, sizeof(addressJump));
	memset(addressMSB, 0, sizeof(addressMSB));

	while (fscanf(fp, ":%2s%4s%2s%s\n", hexByteCount, hexAddressLSB, hexRecordType, hexData) != EOF) {
		unsigned int byteCount, addressLSB, recordType;

		recordType = stmHexToChar(hexRecordType);
		hexData[stmHexToChar(hexByteCount) * 2] = 0;	// terminate at CHKSUM

//		printf("Record Type: %d\n", recordType);
		switch (recordType) {
			case 0x00:
				stmWriteCommand(s, addressMSB, hexAddressLSB, hexByteCount, hexData);
				break;
			case 0x01:
				// EOF
				return addressJump;
				break;
			case 0x04:
				// MSB of destination 32 bit address
				strncpy(addressMSB, hexData, 4);
				break;
			case 0x05:
				// 32 bit address to run after load
				strncpy(addressJump, hexData, 8);
				break;
		}
	}

	return 0;
}

void stmLoader(serialStruct_t *s, FILE *fp, unsigned char overrideParity, unsigned char noSendR) {
	char c;
	unsigned char b1, b2, b3;
	unsigned char i, n;
	char *jumpAddress;

	// turn on parity generation
	if (!overrideParity)
		serialEvenParity(s);

	if(!noSendR) {
		top:
		printf("Sending R to place Baseflight in bootloader, press a key to continue");
		serialFlush(s);
		c = 'R';
		serialWrite(s, &c, 1);
		getchar();
		printf("\n");
	}

	serialFlush(s);

	printf("Poking the MCU to check whether bootloader is alive...");

	// poke the MCU
	do {
		printf("p"); fflush(stdout);
		c = 0x7f;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_SHORT));
	printf("STM bootloader alive...\n");

	// send GET command
	do {
		c = 0x00;
		serialWrite(s, &c, 1);
		c = 0xff;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	b1 = serialRead(s);	// number of bytes
	b2 = serialRead(s);	// bootloader version

	for (i = 0; i < b1; i++)
		getResults[i] = serialRead(s);

	stmWaitAck(s, STM_RETRIES_LONG);
	printf("Received commands.\n");


	// send GET VERSION command
	do {
		c = getResults[1];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));
	b1 = serialRead(s);
	b2 = serialRead(s);
	b3 = serialRead(s);
	stmWaitAck(s, STM_RETRIES_LONG);
	printf("STM Bootloader version: %d.%d\n", (b1 & 0xf0) >> 4, (b1 & 0x0f));

	// send GET ID command
	do {
		c = getResults[2];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));
	n = serialRead(s);
	printf("STM Device ID: 0x");
	for (i = 0; i <= n; i++) {
		b1 = serialRead(s);
		printf("%02x", b1);
	}
	stmWaitAck(s, STM_RETRIES_LONG);
	printf("\n");

/*
	flash_size:
	// read Flash size
	c = getResults[3];
	serialWrite(s, &c, 1);
	c = 0xff ^ c;
	serialWrite(s, &c, 1);

	// if read not allowed, unprotect (which also erases)
	if (!stmWaitAck(s, STM_RETRIES_LONG)) {
		// unprotect command
		do {
			c = getResults[10];
			serialWrite(s, &c, 1);
			c = 0xff ^ c;
			serialWrite(s, &c, 1);
		} while (!stmWaitAck(s, STM_RETRIES_LONG));

		// wait for results
		if (stmWaitAck(s, STM_RETRIES_LONG))
			goto top;
	}

	// send address
	if (!stmWrite(s, "1FFFF7E0"))
		goto flash_size;

	// send # bytes (N-1 = 1)
	if (!stmWrite(s, "01"))
		goto flash_size;

	b1 = serialRead(s);
	b2 = serialRead(s);
	printf("STM Flash Size: %dKB\n", b2<<8 | b1);
*/

	// erase flash
	erase_flash:
	printf("Global flash erase [command 0x%x]...", getResults[6]); fflush(stdout);
	do {
		c = getResults[6];
		serialWrite(s, &c, 1);
		c = 0xff ^ c;
		serialWrite(s, &c, 1);
	} while (!stmWaitAck(s, STM_RETRIES_LONG));

	// global erase
	if (getResults[6] == 0x44) {
		// mass erase
		if (!stmWrite(s, "FFFF"))
			goto erase_flash;
	}
	else {
		c = 0xff;
		serialWrite(s, &c, 1);
		c = 0x00;
		serialWrite(s, &c, 1);

		if (!stmWaitAck(s, STM_RETRIES_LONG))
			goto erase_flash;
	}

	printf("Done.\n");

	// upload hex file
	printf("Flashing device...\n");
	jumpAddress = stmHexLoader(s, fp);
	if (jumpAddress) {
		printf("\nFlash complete, executing.\n");

		go:
		// send GO command
		do {
			c = getResults[4];
			serialWrite(s, &c, 1);
			c = 0xff ^ c;
			serialWrite(s, &c, 1);
		} while (!stmWaitAck(s, STM_RETRIES_LONG));

		// send address
		if (!stmWrite(s, jumpAddress))
			goto go;
	}
	else {
		printf("\nFlash complete.\n");
	}
}
