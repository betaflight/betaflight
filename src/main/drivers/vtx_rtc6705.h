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

/*
 * Author: Giles Burgess (giles@multiflite.co.uk)
 *
 * This source code is provided as is and can be used/modified so long
 * as this header is maintained with the file at all times.
 */

#pragma once

#include <stdint.h>

#define RTC6705_BAND_MIN    1
#define RTC6705_BAND_MAX    5
#define RTC6705_CHANNEL_MIN 1
#define RTC6705_CHANNEL_MAX 8
#define RTC6705_FREQ_MIN    5600
#define RTC6705_FREQ_MAX    5950

bool rtc6705Init(void);
void rtc6705SetChannel(uint8_t band, uint8_t channel);
void rtc6705SetFreq(uint16_t freq);
