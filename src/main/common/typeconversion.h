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

void uli2a(unsigned long int num, unsigned int base, int uc, char *bf);
void li2a(long num, char *bf);
void ui2a(unsigned int num, unsigned int base, int uc, char *bf);
void i2a(int num, char *bf);
char a2i(char ch, const char **src, int base, int *nump);

/* Simple conversion of a float to a string. Will display completely
 * inaccurate results for floats larger than about 2.15E6 (2^31 / 1000)
 * (same thing for negative values < -2.15E6).
 * Will always display 3 decimals, so anything smaller than 1E-3 will
 * not be displayed.
 * The floatString will be filled in with the result and will be
 * returned. It must be at least 13 bytes in length to cover all cases!
 */
char *ftoa(float x, char *floatString);

float fastA2F(const char *p);

#ifndef HAVE_ITOA_FUNCTION
char *itoa(int i, char *a, int r);
#endif
