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

#include <stdbool.h>
#include <stdint.h>

#define HX711_CONFIGURED 1
#define HX711_FRESH 2
#define HX711_SATURATED 4

typedef struct hx711Sample_s {
    uint8_t flags;
    int32_t raw;
    uint32_t sequence;
    uint32_t ageMs;
    uint32_t timeUs;
} hx711Sample_t;

void hx711Init(void);
void hx711Update(void);
void hx711GetSample(hx711Sample_t *sample);
