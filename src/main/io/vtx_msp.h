/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "build/build_config.h"

typedef enum {
    // Offline - device hasn't responded yet
    MSP_VTX_STATUS_OFFLINE = 0,
    MSP_VTX_STATUS_READY,
} mspVtxStatus_e;

bool vtxMspInit(void);
void setMspVtxDeviceStatusReady(const int descriptor);
void prepareMspFrame(uint8_t *mspFrame);
