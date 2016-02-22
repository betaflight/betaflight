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
 * telemetry.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */
#include "rx/rx.h"

#ifndef TELEMETRY_COMMON_H_
#define TELEMETRY_COMMON_H_

typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;

typedef enum {
    FRSKY_UNIT_METRICS = 0,
    FRSKY_UNIT_IMPERIALS
} frskyUnit_e;

typedef struct telemetryConfig_s {
    uint8_t telemetry_switch;               // Use aux channel to change serial output & baudrate( MSP / Telemetry ). It disables automatic switching to Telemetry when armed.
    uint8_t telemetry_inversion;            // also shared with smartport inversion
    float gpsNoFixLatitude;   
    float gpsNoFixLongitude;  
    frskyGpsCoordFormat_e frsky_coordinate_format;   
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t frsky_vfas_cell_voltage;
    uint8_t hottAlarmSoundInterval;
} telemetryConfig_t;

void telemetryCheckState(void);
void telemetryProcess(rxConfig_t *rxConfig, uint16_t deadband3d_throttle);

bool telemetryDetermineEnabledState(portSharing_e portSharing);

void telemetryUseConfig(telemetryConfig_t *telemetryConfig);

#endif /* TELEMETRY_COMMON_H_ */
