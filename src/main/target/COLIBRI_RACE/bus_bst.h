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

#define TBS_CORE_PNP_PRO                    0x80
#define RESERVED                            0x8A
#define PNP_PRO_DIDITAL_CURRENT_SENSOR      0xC0
#define PNP_PRO_GPS                         0xC2
#define TSB_BLACKBOX                        0xC4
#define CLEANFLIGHT_FC                      0xC8
#define CROSSFIRE_UHF_RECEIVER              0xEC

#define GPS_POSITION_FRAME_ID               0x02
#define GPS_TIME_FRAME_ID                   0x03
#define FC_ATTITUDE_FRAME_ID                0x1E
#define RC_CHANNEL_FRAME_ID                 0x15
#define CROSSFIRE_RSSI_FRAME_ID             0x14
#define CLEANFLIGHT_MODE_FRAME_ID           0x20

#define DATA_BUFFER_SIZE                      64

typedef enum BSTDevice {
    BSTDEV_1,
    BSTDEV_2,
    BSTDEV_MAX = BSTDEV_2,
} BSTDevice;

void bstInit(BSTDevice index);
uint32_t bstTimeoutUserCallback(void);
uint16_t bstGetErrorCounter(void);

bool bstWriteBusy(void);
bool bstMasterWrite(uint8_t* data);
bool bstSlaveRead(uint8_t* buf);
bool bstSlaveWrite(uint8_t* data);

void bstMasterWriteLoop(void);

void crc8Cal(uint8_t data_in);

