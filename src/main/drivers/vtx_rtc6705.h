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

#define RTC6705_BAND_COUNT          5
#define RTC6705_CHANNEL_COUNT       8
#define RTC6705_RF_POWER_COUNT      2

#define RTC6705_FREQ_MIN            5600
#define RTC6705_FREQ_MAX            5950

#define RTC6705_BOOT_DELAY 350 // milliseconds

void rtc6705IOInit(void);
void rtc6705SetBandAndChannel(const uint8_t band, const uint8_t channel);
void rtc6705SetFreq(const uint16_t freq);
void rtc6705SetRFPower(const uint8_t rf_power);
void rtc6705Disable(void);
void rtc6705Enable(void);
