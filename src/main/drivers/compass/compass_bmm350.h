/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * BMM350 magnetometer driver (I2C only).
 *
 * Magnetic reset (BR/FGR) requires a 2.2 uF capacitor on CRST. Without it,
 * heading accuracy is degraded. Addresses 0x14 (ADSEL=GND) and 0x15 (ADSEL=VDDIO).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/compass/compass.h"

typedef struct bmm350Comp_s {
    float offsetX;
    float offsetY;
    float offsetZ;
    float offsetTemp;
    float sensitX;
    float sensitY;
    float sensitZ;
    float sensitTemp;
    float tcoX;
    float tcoY;
    float tcoZ;
    float tcsX;
    float tcsY;
    float tcsZ;
    float t0;
    float crossXY;
    float crossYX;
    float crossZX;
    float crossZY;
} bmm350Comp_t;

typedef enum {
    BMM350_RATE_100_HZ = 100,
    BMM350_RATE_400_HZ = 400,
} bmm350Rate_e;

typedef enum {
    BMM350_SAMPLE_ERROR = -1,
    BMM350_SAMPLE_PENDING = 0,
    BMM350_SAMPLE_READY = 1,
} bmm350ReadResult_e;

typedef struct bmm350State_s {
    bmm350Comp_t comp;
    uint8_t receiveBuffer[14]; // Two I2C dummy bytes and four 24-bit channels.
    bmm350Rate_e rate;
    bool readInProgress;
    bool trimValid;
    bool magneticResetComplete;
    bool pollingReady;
} bmm350State_t;

/* Keep state alive; detection and initialization require no outstanding read. */
bool bmm350DetectWithState(magDev_t *magDev, bmm350State_t *state, bmm350Rate_e rate);

/* Start or finish an asynchronous acquisition. Results are in uT and deg C. */
bool bmm350ReadSample(magDev_t *magDev, float magnetic[3], float *temperature);

/* Require valid OTP, magnetic reset and the exact rate/averaging profile, then
 * configure latched DRDY for a poller without an external interrupt pin. */
bool bmm350InitForPolling(magDev_t *magDev, bmm350State_t *state, bmm350Rate_e rate);

/* Finish synchronous transfers immediately, or retain an asynchronous read.
 * Outputs are unchanged while pending or on error. */
bmm350ReadResult_e bmm350PollSample(magDev_t *magDev, float magnetic[3], float *temperature);

/**
 * @brief Detect a BMM350 on I2C and install init/read callbacks.
 *
 * @param magDev Magnetometer device (address 0 probes 0x14 then 0x15).
 * @return true if CHIP_ID 0x33 was read.
 */
bool bmm350Detect(magDev_t *magDev);
