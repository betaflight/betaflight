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

#include "common/time.h"

typedef struct {
    uint8_t dataAge;
    int8_t temperature;
    int16_t voltage;
    int16_t current;
    int16_t consumption;
    int16_t rpm;
} escSensorData_t;

#define ESC_DATA_INVALID 255

bool escSensorInit(void);
void escSensorProcess(timeUs_t currentTime);

#define ESC_SENSOR_COMBINED 255

escSensorData_t *getEscSensorData(uint8_t motorNumber);

