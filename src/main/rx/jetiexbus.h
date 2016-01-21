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


#include "rx/rx.h"

bool jetiExBusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback);
uint8_t jetiExBusFrameStatus(void);


#ifdef TELEMETRY

#include "telemetry/telemetry.h"

void initJetiExBusTelemetry(telemetryConfig_t *initialTelemetryConfig);
void checkJetiExBusTelemetryState(void);
void handleJetiExBusTelemetry(void);

#endif //TELEMETRY
