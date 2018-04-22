/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software: you can redistribute 
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

#include "drivers/io_types.h"

typedef enum softSPIDevice {
    SOFT_SPIDEV_1   = 0
} softSPIDevice_e;

typedef struct softSPIDevice_s {
    ioTag_t sckTag;
    ioTag_t mosiTag;
    ioTag_t misoTag;
    ioTag_t nssTag;
} softSPIDevice_t;


void softSpiInit(const softSPIDevice_t *dev);
uint8_t softSpiTransferByte(const softSPIDevice_t *dev, uint8_t data);
