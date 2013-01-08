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

#ifndef _serial_h
#define _serial_h

#define INPUT_BUFFER_SIZE	1024

typedef struct {
	int fd;
} serialStruct_t;

extern serialStruct_t *initSerial(const char *port, unsigned int baud, char ctsRts);
extern void serialWrite(serialStruct_t *s, const char *str, unsigned int len);
extern void serialWriteChar(serialStruct_t *s, unsigned char c);
extern unsigned char serialAvailable(serialStruct_t *s);
extern void serialFlush(serialStruct_t *s);
extern unsigned char serialRead(serialStruct_t *s);
extern void serialEvenParity(serialStruct_t *s);
extern void serialNoParity(serialStruct_t *s);
extern void serialFree(serialStruct_t *s);

#endif
