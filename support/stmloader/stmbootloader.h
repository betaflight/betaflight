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

#ifndef _stmbootloader_h
#define _stmbootloader_h

#include <stdio.h>
#include "serial.h"

extern void stmLoader(serialStruct_t *s, FILE *fp, unsigned char overrideParity, unsigned char noSendR);

#endif
