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

/*
 * The boot ROM API table layout and the FlexSPI NOR driver interface are based
 * on lib/modules/mcux-devices-rt/RT1060/MIMXRT1062/drivers/romapi/fsl_romapi.h
 * Copyright 2017-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Include after platform.h, which defines RAMFUNC and IMXRT_BOOTLOADER_TREE_ADDR.

#pragma once

#include <stdint.h>

#include "xip/flexspi_nor_config.h"

// The ROM driver reports success as zero.
#define ROM_STATUS_SUCCESS          0

typedef int32_t romStatus_t;

// FlexSPI NOR driver exported by the boot ROM; the 11xx ROM leaves clear_cache null.
typedef struct flexspiNorDriverInterface_s {
    uint32_t version;
    romStatus_t (*init)(uint32_t instance, flexspiNorConfig_t *config);
    romStatus_t (*program)(uint32_t instance, flexspiNorConfig_t *config, uint32_t dstAddr, const uint32_t *src);
    romStatus_t (*erase_all)(uint32_t instance, flexspiNorConfig_t *config);
    romStatus_t (*erase)(uint32_t instance, flexspiNorConfig_t *config, uint32_t start, uint32_t lengthInBytes);
    romStatus_t (*read)(uint32_t instance, flexspiNorConfig_t *config, uint32_t *dst, uint32_t addr,
                        uint32_t lengthInBytes);
    void (*clear_cache)(uint32_t instance);
    romStatus_t (*xfer)(uint32_t instance, void *xfer);
    romStatus_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
    romStatus_t (*get_config)(uint32_t instance, flexspiNorConfig_t *config, void *option);
} flexspiNorDriverInterface_t;

// The ROM's FlexSPI NOR driver, or NULL if the part has no usable ROM table.
RAMFUNC const flexspiNorDriverInterface_t *romFlexspiNorDriver(void);

// Enter the ROM serial downloader; returns only if the ROM table is missing.
void romRunSerialDownloader(void);
