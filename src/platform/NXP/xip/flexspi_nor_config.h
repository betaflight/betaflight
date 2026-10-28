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
 * Structure layout, lookup table encoding and enumerated field values based on
 * fsl_romapi.h from the NXP MCUXpresso SDK, in the submodule at
 * lib/modules/mcux-devices-rt/RT1060/MIMXRT1062/drivers/romapi/fsl_romapi.h
 * Copyright 2017-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// FlexSPI configuration block the boot ROM reads from the start of flash; every field offset is fixed by the ROM.

#pragma once

#include <stdint.h>

#include "common/utils.h"

// The block version differs per family; the 10xx ROM accepts V1.0.0 and up.
#define FLEXSPI_CFG_BLK_TAG         0x42464346u   /* "FCFB" as read back little endian */
#if defined(IMXRT11XX)
#define FLEXSPI_CFG_BLK_VERSION     0x56010400u   /* "V" 1.4.0 */
#elif defined(IMXRT10XX)
#define FLEXSPI_CFG_BLK_VERSION     0x56010100u   /* "V" 1.1.0 */
#else
#error "Unsupported i.MXRT family: no FlexSPI configuration block version"
#endif
#define FLEXSPI_CFG_BLK_SIZE        512

/* LUT instruction opcodes. The DDR forms drive an octal part. */
#define CMD_SDR                     0x01
#define CMD_DDR                     0x21
#define RADDR_SDR                   0x02
#define RADDR_DDR                   0x22
#define WRITE_SDR                   0x08
#define WRITE_DDR                   0x28
#define READ_SDR                    0x09
#define READ_DDR                    0x29
#define DUMMY_SDR                   0x0c
#define DUMMY_DDR                   0x2c
#define STOP                        0x00

/* Number of pads an instruction uses */
#define FLEXSPI_1PAD                0
#define FLEXSPI_2PAD                1
#define FLEXSPI_4PAD                2
#define FLEXSPI_8PAD                3

/* A LUT word holds two instructions: opcode[5:0], pads[1:0], operand[7:0] */
#define FLEXSPI_LUT_INSTR(cmd, pad, op) \
    ((uint32_t)(((uint32_t)(cmd) << 10) | ((uint32_t)(pad) << 8) | ((uint32_t)(op) & 0xffu)))
#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1) \
    (FLEXSPI_LUT_INSTR(cmd0, pad0, op0) | (FLEXSPI_LUT_INSTR(cmd1, pad1, op1) << 16))

/* Read sample clock source */
#define FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY       0
#define FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD      1
#define FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD      2
#define FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD 3

// Serial clock frequency codes, numbered differently per family; the 11xx list
// is the 117x one, which the vendor 1176 ROM API header does not match.
#if defined(IMXRT10XX)
#define FLEXSPI_SERIAL_CLKFREQ_30MHZ    1
#define FLEXSPI_SERIAL_CLKFREQ_50MHZ    2
#define FLEXSPI_SERIAL_CLKFREQ_60MHZ    3
#define FLEXSPI_SERIAL_CLKFREQ_75MHZ    4
#define FLEXSPI_SERIAL_CLKFREQ_80MHZ    5
#define FLEXSPI_SERIAL_CLKFREQ_100MHZ   6
#define FLEXSPI_SERIAL_CLKFREQ_133MHZ   7
#define FLEXSPI_SERIAL_CLKFREQ_166MHZ   8
#define FLEXSPI_SERIAL_CLKFREQ_200MHZ   9
#elif defined(IMXRT11XX)
#define FLEXSPI_SERIAL_CLKFREQ_30MHZ    1
#define FLEXSPI_SERIAL_CLKFREQ_50MHZ    2
#define FLEXSPI_SERIAL_CLKFREQ_60MHZ    3
#define FLEXSPI_SERIAL_CLKFREQ_80MHZ    4
#define FLEXSPI_SERIAL_CLKFREQ_100MHZ   5
#define FLEXSPI_SERIAL_CLKFREQ_120MHZ   6
#define FLEXSPI_SERIAL_CLKFREQ_133MHZ   7
#define FLEXSPI_SERIAL_CLKFREQ_166MHZ   8
#define FLEXSPI_SERIAL_CLKFREQ_200MHZ   9
#else
#error "Unsupported i.MXRT family: no FlexSPI serial clock frequency codes"
#endif

/* Attached device type */
#define FLEXSPI_DEVICE_TYPE_SERIAL_NOR  1

/* deviceModeType: what the device mode command sent at startup does */
#define FLEXSPI_DEVICE_CONFIG_CMD_TYPE_GENERIC      0
#define FLEXSPI_DEVICE_CONFIG_CMD_TYPE_SPI2XPI      2

/* Bit numbers within controllerMiscOption */
#define FLEXSPI_MISC_OFFSET_SAFE_CONFIG_FREQ_ENABLE 4
#define FLEXSPI_MISC_OFFSET_DDR_MODE_ENABLE         6

/* Number of pads wired to the device */
#define SERIAL_FLASH_1PAD               1
#define SERIAL_FLASH_2PADS              2
#define SERIAL_FLASH_4PADS              4
#define SERIAL_FLASH_8PADS              8

typedef struct flexspiLutSeq_s {
    uint8_t seqNum;             /* number of LUT words in the sequence */
    uint8_t seqId;              /* index of the first LUT word */
    uint16_t reserved;
} flexspiLutSeq_t;

typedef struct flexspiMemConfig_s {
    uint32_t tag;
    uint32_t version;
    uint32_t reserved0;
    uint8_t readSampleClkSrc;
    uint8_t csHoldTime;
    uint8_t csSetupTime;
    uint8_t columnAddressWidth;
    uint8_t deviceModeCfgEnable;
    uint8_t deviceModeType;
    uint16_t waitTimeCfgCommands;
    flexspiLutSeq_t deviceModeSeq;
    uint32_t deviceModeArg;
    uint8_t configCmdEnable;
    uint8_t configModeType[3];
    flexspiLutSeq_t configCmdSeqs[3];
    uint32_t reserved1;
    uint32_t configCmdArgs[3];
    uint32_t reserved2;
    uint32_t controllerMiscOption;
    uint8_t deviceType;
    uint8_t sflashPadType;
    uint8_t serialClkFreq;
    uint8_t lutCustomSeqEnable;
    uint32_t reserved3[2];
    uint32_t sflashA1Size;
    uint32_t sflashA2Size;
    uint32_t sflashB1Size;
    uint32_t sflashB2Size;
    uint32_t csPadSettingOverride;
    uint32_t sclkPadSettingOverride;
    uint32_t dataPadSettingOverride;
    uint32_t dqsPadSettingOverride;
    uint32_t timeoutInMs;
    uint32_t commandInterval;
    uint16_t dataValidTime[2];
    uint16_t busyOffset;
    uint16_t busyBitPolarity;
    uint32_t lookupTable[64];
    flexspiLutSeq_t lutCustomSeq[12];
    uint32_t reserved4[4];
} flexspiMemConfig_t;

typedef struct flexspiNorConfig_s {
    flexspiMemConfig_t memConfig;
    uint32_t pageSize;
    uint32_t sectorSize;
    uint8_t ipcmdSerialClkFreq;
    uint8_t isUniformBlockSize;
    uint8_t reserved0[2];
    uint8_t serialNorType;
    uint8_t needExitNoCmdMode;
    uint8_t halfClkForNonReadCmd;
    uint8_t needRestoreNoCmdMode;
    uint32_t blockSize;
    uint32_t reserved1[11];
} flexspiNorConfig_t;

/* The ROM reads a fixed 512 byte block whose first 448 bytes are the memory configuration. */
STATIC_ASSERT(sizeof(flexspiMemConfig_t) == 448,
              "FlexSPI memory configuration must be exactly 448 bytes");
STATIC_ASSERT(sizeof(flexspiNorConfig_t) == FLEXSPI_CFG_BLK_SIZE,
              "FlexSPI NOR configuration block must be exactly 512 bytes");

/* The block the boot ROM reads from flash, reused by flexspi_nor_imxrt.c. */
extern const flexspiNorConfig_t flexspiNorConfig;
