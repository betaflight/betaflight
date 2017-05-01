/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

int blackboxPrintf(const char *fmt, ...);
void blackboxPrintfHeaderLine(const char *name, const char *fmt, ...);
int blackboxPrint(const char *s);

void blackboxWriteUnsignedVB(uint32_t value);
void blackboxWriteSignedVB(int32_t value);
void blackboxWriteSignedVBArray(int32_t *array, int count);
void blackboxWriteSigned16VBArray(int16_t *array, int count);
void blackboxWriteS16(int16_t value);
void blackboxWriteTag2_3S32(int32_t *values);
void blackboxWriteTag8_4S16(int32_t *values);
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount);
void blackboxWriteU32(int32_t value);
void blackboxWriteFloat(float value);
