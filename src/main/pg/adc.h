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

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/io_types.h"

typedef struct adcChannelConfig_t {
    bool enabled;
    ioTag_t ioTag;
} adcChannelConfig_t;

typedef struct adcConfig_s {
    adcChannelConfig_t vbat;
    adcChannelConfig_t rssi;
    adcChannelConfig_t current;
    adcChannelConfig_t external1;
    int8_t device; // ADCDevice
} adcConfig_t;

PG_DECLARE(adcConfig_t, adcConfig);
