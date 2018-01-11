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

#include "rx/rx_spi.h"

#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0

#define SYNC_DELAY_MAX 9000

#define MAX_MISSING_PKT 100

enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING,
    STATE_BIND_BINDING1,
    STATE_BIND_BINDING2,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};

extern uint8_t listLength;
extern uint32_t missingPackets;
extern timeDelta_t timeoutUs;
extern int16_t rssiDbm;

extern IO_t gdoPin;

void setRssiDbm(uint8_t value);

void TxEnable(void);
void TxDisable(void);

void LedOn(void);
void LedOff(void);

void switchAntennae(void);

void initialiseData(uint8_t adr);

bool checkBindRequested(bool reset);

void nextChannel(uint8_t skip);
