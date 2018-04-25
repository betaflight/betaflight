/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FTOA_BUFFER_LENGTH 11

void uli2a(unsigned long int num, unsigned int base, int uc, char *bf);
void li2a(long num, char *bf);
void ui2a(unsigned int num, unsigned int base, int uc, char *bf);
void i2a(int num, char *bf);
char a2i(char ch, const char **src, int base, int *nump);
char *ftoa(float x, char *floatString);
float fastA2F(const char *p);

#ifndef HAVE_ITOA_FUNCTION
char *itoa(int i, char *a, int r);
#endif
