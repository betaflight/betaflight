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

typedef enum {
    FRSKY_VFAS_PRECISION_LOW = 0,
    FRSKY_VFAS_PRECISION_HIGH
} frskyVFasPrecision_e;

typedef void frSkyHubWriteByteFn(const char data);

void handleFrSkyHubTelemetry(timeUs_t currentTimeUs);
void checkFrSkyHubTelemetryState(void);

bool initFrSkyHubTelemetry(void);
bool initFrSkyHubTelemetryExternal(frSkyHubWriteByteFn *frSkyWriteFrameExternal);

void processFrSkyHubTelemetry(timeUs_t currentTimeUs);
