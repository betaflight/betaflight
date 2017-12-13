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

#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0

#define SYNC 9000

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

extern bool bindRequested;
extern uint8_t listLength;
extern int16_t RSSI_dBm;

extern IO_t gdoPin;
extern IO_t frSkyLedPin;
extern IO_t antSelPin;

void setRssiDbm(uint8_t value);

void frskySpiRxSetup();

void RxEnable(void);
void TxEnable(void);

void initialiseData(uint8_t adr);

bool checkBindRequested(bool reset);

void handleBinding(uint8_t protocolState, uint8_t *packet);

void nextChannel(uint8_t skip, bool sendStrobe);
