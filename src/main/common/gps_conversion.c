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

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#ifdef USE_GPS

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(const char* coordinateString)
{
    const char *fieldSeparator, *remainingString;
    uint8_t degrees = 0, minutes = 0;
    uint16_t fractionalMinutes = 0;
    uint8_t digitIndex;

    // scan for decimal point or end of field
    for (fieldSeparator = coordinateString; isdigit((unsigned char)*fieldSeparator); fieldSeparator++) {
        if (fieldSeparator >= coordinateString + 15)
            return 0; // stop potential fail
    }
    remainingString = coordinateString;

    // convert degrees
    while ((fieldSeparator - remainingString) > 2) {
        if (degrees)
            degrees *= 10;
        degrees += DIGIT_TO_VAL(*remainingString++);
    }
    // convert minutes
    while (fieldSeparator > remainingString) {
        if (minutes)
            minutes *= 10;
        minutes += DIGIT_TO_VAL(*remainingString++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*fieldSeparator == '.') {
        remainingString = fieldSeparator + 1;
        for (digitIndex = 0; digitIndex < 4; digitIndex++) {
            fractionalMinutes *= 10;
            if (isdigit((unsigned char)*remainingString))
                fractionalMinutes += *remainingString++ - '0';
        }
    }
    return degrees * 10000000UL + (minutes * 1000000UL + fractionalMinutes * 100UL) / 6;
}
#endif
