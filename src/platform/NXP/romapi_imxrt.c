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
 * The boot ROM API table layout and the fixed table address are based on
 * lib/modules/mcux-devices-rt/RT1060/MIMXRT1062/drivers/romapi/fsl_romapi.c
 * Copyright 2017-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "romapi_imxrt.h"

// runBootloader argument (reference manual, System Boot): tag 0xeb, boot mode 1
// (serial downloader), interface 0 (USB or LPUART, whichever speaks first).
#define ROM_RUN_SERIAL_DOWNLOADER   0xeb100000u

// Root of the boot ROM API table; the member order differs per family.
#if defined(IMXRT11XX)
typedef struct bootloaderApiEntry_s {
    void (*runBootloader)(void *arg);
    const uint32_t version;
    const char *copyright;
    const flexspiNorDriverInterface_t *flexSpiNorDriver;
    const uint32_t reserved[8];
} bootloaderApiEntry_t;

// The A0 silicon keeps its table pointer 64 kB below later revisions.
#define ANADIG_CHIPID_1170_A0       0x001170a0u
#define ROM_TREE_ADDR_1170_A0       0x0020001cu
#define ROM_TREE_ADDR_1170_B0       0x0021001cu
#elif defined(IMXRT10XX)
typedef struct bootloaderApiEntry_s {
    const uint32_t version;
    const char *copyright;
    void (*runBootloader)(void *arg);
    const uint32_t *reserved0;
    const flexspiNorDriverInterface_t *flexSpiNorDriver;
    const uint32_t *reserved1[5];
} bootloaderApiEntry_t;
#else
#error "Unsupported i.MXRT family: no boot ROM API table layout"
#endif

// Inline, so the flash writer's copy lands in ITCM with its caller.
static inline const bootloaderApiEntry_t *romApiTree(void)
{
#if defined(IMXRT11XX)
    // Read on every call: the flash writer runs while flash is unavailable.
    const uintptr_t treeAddr = (ANADIG_MISC->MISC_DIFPROG == ANADIG_CHIPID_1170_A0)
                             ? ROM_TREE_ADDR_1170_A0 : ROM_TREE_ADDR_1170_B0;
#elif defined(IMXRT10XX)
    const uintptr_t treeAddr = IMXRT_BOOTLOADER_TREE_ADDR;
#else
#error "Unsupported i.MXRT family: no boot ROM API table address"
#endif

    return *(const bootloaderApiEntry_t *const *)treeAddr;
}

RAMFUNC const flexspiNorDriverInterface_t *romFlexspiNorDriver(void)
{
    const bootloaderApiEntry_t *apiTree = romApiTree();

    return apiTree ? apiTree->flexSpiNorDriver : NULL;
}

void romRunSerialDownloader(void)
{
    const bootloaderApiEntry_t *apiTree = romApiTree();

    if (!apiTree || !apiTree->runBootloader) {
        return;
    }

    // Does not return; interrupts stay masked since the ROM may reconfigure the flash.
    uint32_t arg = ROM_RUN_SERIAL_DOWNLOADER;

    __disable_irq();
    apiTree->runBootloader(&arg);
}
