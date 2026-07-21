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
 *
 * Author: Dominic Clifton
 */

#pragma once

#include "drivers/bus_quadspi_types.h"

#ifdef USE_OCTOSPI

typedef enum octoSpiDevice_e {
    OCTOSPIINVALID = -1,
    OCTOSPIDEV_1   = 0,
} octoSpiDevice_e;

#define OCTOSPIDEV_COUNT 1

// Macros to convert between configuration ids and device ids.
#define OCTOSPI_CFG_TO_DEV(x)   ((x) - 1)
#define OCTOSPI_DEV_TO_CFG(x)   ((x) + 1)

octoSpiDevice_e octoSpiDeviceByInstance(octoSpiResource_t *instance);
octoSpiResource_t *octoSpiInstanceByDevice(octoSpiDevice_e device);

bool octoSpiInit(octoSpiDevice_e device);
bool octoSpiReceive1LINE(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);
bool octoSpiReceive4LINES(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);
bool octoSpiTransmit1LINE(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length);

bool octoSpiReceiveWithAddress1LINE(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool octoSpiReceiveWithAddress4LINES(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool octoSpiTransmitWithAddress1LINE(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);
bool octoSpiTransmitWithAddress4LINES(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);

bool octoSpiInstructionWithAddress1LINE(octoSpiResource_t *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize);

void octoSpiDisableMemoryMappedMode(octoSpiResource_t *instance);
void octoSpiEnableMemoryMappedMode(octoSpiResource_t *instance);

#endif
