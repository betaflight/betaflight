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
 */

// Image vector table and boot data the boot ROM reads at flash + 0x1000; the
// layout is fixed by the HAB specification in the reference manual.

#include <stdint.h>

#include "platform.h"

// Absolute linker symbols: the address is the value.
extern uint32_t __flash_start[];
extern uint32_t __flash_image_size[];

// Header word: tag 0xD1, big-endian length 0x2000, HAB version 4.1.
#define IVT_TAG                 0xd1u
#define IVT_LENGTH              0x2000u
#define IVT_VERSION             0x41u
#define IVT_HEADER              (IVT_TAG | (IVT_LENGTH << 8) | (IVT_VERSION << 24))

// The boot ROM looks for the IVT one 4 KB block into the flash.
#define IVT_OFFSET              0x1000u

typedef struct imageVectorTable_s {
    uint32_t header;
    uint32_t entry;         // first instruction to run
    uint32_t reserved1;
    uint32_t dcd;           // device configuration data, 0 when there is none
    uint32_t bootData;
    uint32_t self;          // absolute address of this structure
    uint32_t csf;           // command sequence file, only used by signed images
    uint32_t reserved2;
} imageVectorTable_t;

typedef struct bootData_s {
    uint32_t start;
    uint32_t size;
    uint32_t plugin;
    uint32_t reserved;      // pads the structure out to 16 bytes
} bootData_t;

// The reserved boot data word reads as erased flash.
#define BOOT_DATA_RESERVED      0xffffffffu

// Initial stack pointer and reset address the boot ROM reads through the IVT
// entry field, since the real vector table runs from ITCM.
extern uint32_t __stack_top[];
void Reset_Handler(void);

__attribute__((section(".boot_vector"), used))
const uint32_t bootVector[2] = {
    (uint32_t)__stack_top,
    (uint32_t)&Reset_Handler,
};

__attribute__((section(".boot_hdr.boot_data"), used))
const bootData_t bootData = {
    .start       = (uint32_t)__flash_start,
    // Image size, not part size.
    .size        = (uint32_t)__flash_image_size,
    .plugin      = 0,
    .reserved    = BOOT_DATA_RESERVED,
};

__attribute__((section(".boot_hdr.ivt"), used))
const imageVectorTable_t imageVectorTable = {
    .header    = IVT_HEADER,
    .entry     = (uint32_t)bootVector,
    .reserved1 = 0,
    .dcd       = 0,
    .bootData  = (uint32_t)&bootData,
    .self      = (uint32_t)__flash_start + IVT_OFFSET,
    .csf       = 0,
    .reserved2 = 0,
};
