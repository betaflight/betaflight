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

#include "blackbox/blackbox_fielddefs.h"

#include "config/parameter_group.h"

typedef struct blackboxConfig_s {
    uint8_t rate_num;
    uint8_t rate_denom;
    uint8_t device;
    uint8_t invertedCardDetection;
} blackboxConfig_t;

PG_DECLARE(blackboxConfig_t, blackboxConfig);

void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data);

void blackboxInit(void);
void blackboxUpdate(timeUs_t currentTimeUs);
const char *blackboxGetStartDateTime(void);
void blackboxSetStartDateTime(const char *dateTime, timeMs_t timeNowMs);
void blackboxStart(void);
void blackboxFinish(void);
bool blackboxMayEditConfig(void);
