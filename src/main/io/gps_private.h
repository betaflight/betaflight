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

#ifdef GPS

#include "io/serial.h"

#define GPS_HDOP_TO_EPH_MULTIPLIER      2   // empirical value

typedef enum {
    GPS_UNKNOWN,                // 0
    GPS_INITIALIZING,           // 1
    GPS_CHANGE_BAUD,            // 2
    GPS_CHECK_VERSION,          // 3
    GPS_CONFIGURE,              // 4
    GPS_RECEIVING_DATA,         // 5
    GPS_LOST_COMMUNICATION,     // 6
} gpsState_e;

typedef struct {
    const gpsConfig_t *   gpsConfig;
    const serialConfig_t * serialConfig;
    serialPort_t *  gpsPort;                // Serial GPS only

    uint32_t        hwVersion;

    gpsState_e      state;
    gpsBaudRate_e   baudrateIndex;
    gpsBaudRate_e   autoBaudrateIndex;      // Driver internal use (for autoBaud)
    uint8_t         autoConfigStep;         // Driver internal use (for autoConfig)
    uint8_t         autoConfigPosition;     // Driver internal use (for autoConfig)

    uint32_t        lastStateSwitchMs;
    uint32_t        lastLastMessageMs;
    uint32_t        lastMessageMs;
} gpsReceiverData_t;

extern gpsReceiverData_t gpsState;

extern baudRate_e gpsToSerialBaudRate[GPS_BAUDRATE_COUNT];

extern void gpsSetState(gpsState_e state);
extern void gpsFinalizeChangeBaud(void);

extern uint16_t gpsConstrainEPE(uint32_t epe);
extern uint16_t gpsConstrainHDOP(uint32_t hdop);

extern bool gpsHandleNMEA(void);
extern bool gpsHandleMTK(void);
extern bool gpsHandleUBLOX(void);
extern bool gpsHandleI2CNAV(void);
extern bool gpsDetectI2CNAV(void);
extern bool gpsHandleNAZA(void);

#endif
