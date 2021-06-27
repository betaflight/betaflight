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

#ifdef USE_OCTOSPI

typedef enum OCTOSPIDevice {
    OCTOSPIINVALID = -1,
    OCTOSPIDEV_1   = 0,
} OCTOSPIDevice;

#define OCTOSPIDEV_COUNT 1

// Macros to convert between configuration ids and device ids.
#define OCTOSPI_CFG_TO_DEV(x)   ((x) - 1)
#define OCTOSPI_DEV_TO_CFG(x)   ((x) + 1)

#if !defined(STM32H7)
#error OctoSPI unsupported on this MCU
#endif

OCTOSPIDevice octoSpiDeviceByInstance(OCTOSPI_TypeDef *instance);
OCTOSPI_TypeDef *octoSpiInstanceByDevice(OCTOSPIDevice device);


bool octoSpiInit(OCTOSPIDevice device);
bool octoSpiReceive1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);
bool octoSpiReceive4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length);
bool octoSpiTransmit1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length);

bool octoSpiReceiveWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool octoSpiReceiveWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length);
bool octoSpiTransmitWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);
bool octoSpiTransmitWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length);

bool octoSpiInstructionWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize);


void octoSpiDisableMemoryMappedMode(OCTOSPI_TypeDef *instance);
void octoSpiEnableMemoryMappedMode(OCTOSPI_TypeDef *instance);

#endif
