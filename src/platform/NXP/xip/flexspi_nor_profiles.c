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
 * The FlexSPI configuration block the boot ROM reads from flash + 0, one per
 * flash profile. A board may override FLEXSPI_NOR_READ_SAMPLE_CLK_SRC,
 * FLEXSPI_NOR_CS_HOLD_TIME, FLEXSPI_NOR_CS_SETUP_TIME (in serial clock cycles),
 * FLEXSPI_NOR_SERIAL_CLK_FREQ and FLEXSPI_NOR_FAST_SERIAL_CLK_FREQ.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "flexspi_nor_imxrt.h"
#include "xip/flexspi_nor_config.h"
#include "xip/flexspi_nor_profiles.h"

#ifndef FLEXSPI_NOR_PROFILE
#error "Define FLEXSPI_NOR_PROFILE in target.h, or leave this file out of target.mk and supply a flexspi_nor_config.c"
#endif

// Size of the part, from the board's linker script.
extern uint32_t __flash_size[];

// The board wiring knobs and the size, as they go into a memory configuration.
#define FLEXSPI_NOR_BOARD_FIELDS                                    \
    .readSampleClkSrc   = FLEXSPI_NOR_READ_SAMPLE_CLK_SRC,          \
    .csHoldTime         = FLEXSPI_NOR_CS_HOLD_TIME,                 \
    .csSetupTime        = FLEXSPI_NOR_CS_SETUP_TIME,                \
    .serialClkFreq      = FLEXSPI_NOR_SERIAL_CLK_FREQ,              \
    .sflashA1Size       = (uint32_t)__flash_size

#if FLEXSPI_NOR_PROFILE == FLEXSPI_NOR_PROFILE_W25Q

// Winbond W25Q quad NOR: 256 byte pages, 4 KB sectors, 64 KB blocks.
#ifndef FLEXSPI_NOR_READ_SAMPLE_CLK_SRC
#define FLEXSPI_NOR_READ_SAMPLE_CLK_SRC     FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD
#endif
#ifndef FLEXSPI_NOR_CS_HOLD_TIME
#define FLEXSPI_NOR_CS_HOLD_TIME            3
#endif
#ifndef FLEXSPI_NOR_CS_SETUP_TIME
#define FLEXSPI_NOR_CS_SETUP_TIME           3
#endif
#ifndef FLEXSPI_NOR_SERIAL_CLK_FREQ
#define FLEXSPI_NOR_SERIAL_CLK_FREQ         FLEXSPI_SERIAL_CLKFREQ_60MHZ
#endif

/* LUT sequence slots. The ROM has fixed meanings for these indexes. */
enum {
    LUT_READ                = 0,
    LUT_READ_STATUS         = 1,
    LUT_WRITE_ENABLE        = 3,
    LUT_ERASE_SECTOR        = 5,
    LUT_READ_QUAD_OUTPUT    = 6,
    LUT_PAGE_PROGRAM_QUAD   = 7,
    LUT_ERASE_BLOCK         = 8,
    LUT_PAGE_PROGRAM        = 9,
    LUT_CHIP_ERASE          = 11,
};

__attribute__((section(".boot_hdr.conf"), used))
const flexspiNorConfig_t flexspiNorConfig = {
    .memConfig = {
        .tag                = FLEXSPI_CFG_BLK_TAG,
        .version            = FLEXSPI_CFG_BLK_VERSION,
        .columnAddressWidth = 0,
        .deviceType         = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
        .sflashPadType      = SERIAL_FLASH_4PADS,
        FLEXSPI_NOR_BOARD_FIELDS,
        .lookupTable = {
            /* 0xEB quad I/O fast read, 24 bit address on 4 pads, 6 dummy cycles */
            [4 * LUT_READ]                  = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xeb,
                                                              RADDR_SDR, FLEXSPI_4PAD, 0x18),
            [4 * LUT_READ + 1]              = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06,
                                                              READ_SDR, FLEXSPI_4PAD, 0x04),

            /* 0x05 read status register 1 */
            [4 * LUT_READ_STATUS]           = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05,
                                                              READ_SDR, FLEXSPI_1PAD, 0x04),

            /* 0x06 write enable */
            [4 * LUT_WRITE_ENABLE]          = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06,
                                                              STOP, FLEXSPI_1PAD, 0x00),

            /* 0x20 sector erase, 4 KB */
            [4 * LUT_ERASE_SECTOR]          = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20,
                                                              RADDR_SDR, FLEXSPI_1PAD, 0x18),

            /* 0x6B quad output fast read, 8 dummy cycles */
            [4 * LUT_READ_QUAD_OUTPUT]      = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x6b,
                                                              RADDR_SDR, FLEXSPI_1PAD, 0x18),
            [4 * LUT_READ_QUAD_OUTPUT + 1]  = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x08,
                                                              READ_SDR, FLEXSPI_4PAD, 0x04),

            /* 0x32 quad input page program */
            [4 * LUT_PAGE_PROGRAM_QUAD]     = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32,
                                                              RADDR_SDR, FLEXSPI_1PAD, 0x18),
            [4 * LUT_PAGE_PROGRAM_QUAD + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x04,
                                                              STOP, FLEXSPI_1PAD, 0x00),

            /* 0xD8 block erase, 64 KB */
            [4 * LUT_ERASE_BLOCK]           = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xd8,
                                                              RADDR_SDR, FLEXSPI_1PAD, 0x18),

            /* 0x02 single line page program */
            [4 * LUT_PAGE_PROGRAM]          = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02,
                                                              RADDR_SDR, FLEXSPI_1PAD, 0x18),
            [4 * LUT_PAGE_PROGRAM + 1]      = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04,
                                                              STOP, FLEXSPI_1PAD, 0x00),

            /* 0x60 chip erase, never issued: the code runs from this part. */
            [4 * LUT_CHIP_ERASE]            = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60,
                                                              STOP, FLEXSPI_1PAD, 0x00),
        },
    },

    .pageSize           = FLEXSPI_NOR_PAGE_SIZE,
    .sectorSize         = FLEXSPI_NOR_SECTOR_SIZE,
    .blockSize          = 64 * 1024,
    .isUniformBlockSize = false,
};

#elif FLEXSPI_NOR_PROFILE == FLEXSPI_NOR_PROFILE_MX25UM

// Macronix MX25UM octal NOR. The boot block reads it on one pad with 0x03,
// the only read the part answers from reset.
#ifndef FLEXSPI_NOR_READ_SAMPLE_CLK_SRC
#define FLEXSPI_NOR_READ_SAMPLE_CLK_SRC     FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNALLY
#endif
#ifndef FLEXSPI_NOR_CS_HOLD_TIME
#define FLEXSPI_NOR_CS_HOLD_TIME            1
#endif
#ifndef FLEXSPI_NOR_CS_SETUP_TIME
#define FLEXSPI_NOR_CS_SETUP_TIME           1
#endif
#ifndef FLEXSPI_NOR_SERIAL_CLK_FREQ
#define FLEXSPI_NOR_SERIAL_CLK_FREQ         FLEXSPI_SERIAL_CLKFREQ_30MHZ
#endif

// Octal mode clock; the 10xx FlexSPI does not reach 200 MHz.
#ifndef FLEXSPI_NOR_FAST_SERIAL_CLK_FREQ
#define FLEXSPI_NOR_FAST_SERIAL_CLK_FREQ    FLEXSPI_SERIAL_CLKFREQ_200MHZ
#endif

__attribute__((section(".boot_hdr.conf"), used))
const flexspiNorConfig_t flexspiNorConfig = {
    .memConfig = {
        .tag                  = FLEXSPI_CFG_BLK_TAG,
        .version              = FLEXSPI_CFG_BLK_VERSION,
        .deviceModeCfgEnable  = 1,
        .deviceModeType       = FLEXSPI_DEVICE_CONFIG_CMD_TYPE_GENERIC,
        .waitTimeCfgCommands  = 1,
        // Let the ROM configure the part at a safe frequency first.
        .controllerMiscOption = 1u << FLEXSPI_MISC_OFFSET_SAFE_CONFIG_FREQ_ENABLE,
        .deviceType           = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
        .sflashPadType        = SERIAL_FLASH_1PAD,
        FLEXSPI_NOR_BOARD_FIELDS,
        .lookupTable = {
            /* 0x03 read, 24 bit address, one pad */
            [0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03,
                                  RADDR_SDR, FLEXSPI_1PAD, 0x18),
            [1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04,
                                  STOP, FLEXSPI_1PAD, 0x00),
        },
    },

    .pageSize           = FLEXSPI_NOR_PAGE_SIZE,
    .sectorSize         = FLEXSPI_NOR_SECTOR_SIZE,
    .ipcmdSerialClkFreq = 1,
    .isUniformBlockSize = false,
    .serialNorType      = 2,
    .blockSize          = 64 * 1024,
    // The 11xx ROM reads the first reserved word as the flash timing option.
    .reserved1          = { 0x07008200 },
};

// The same part in octal DDR, handed to the ROM driver by flexspiNorInit().
const flexspiNorConfig_t flexspiNorFastConfig = {
    .memConfig = {
        .tag                  = FLEXSPI_CFG_BLK_TAG,
        .version              = FLEXSPI_CFG_BLK_VERSION,
        .readSampleClkSrc     = FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD,
        .csHoldTime           = 1,
        .csSetupTime          = 1,
        .deviceModeCfgEnable  = 1,
        .deviceModeType       = FLEXSPI_DEVICE_CONFIG_CMD_TYPE_SPI2XPI,
        .waitTimeCfgCommands  = 1,
        // Sequence 6 writes 2 to configuration register 2 for octal DDR.
        .deviceModeSeq        = { .seqNum = 1, .seqId = 6 },
        .deviceModeArg        = 2,
        .controllerMiscOption = (1u << FLEXSPI_MISC_OFFSET_SAFE_CONFIG_FREQ_ENABLE)
                              | (1u << FLEXSPI_MISC_OFFSET_DDR_MODE_ENABLE),
        .deviceType           = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
        .sflashPadType        = SERIAL_FLASH_8PADS,
        .serialClkFreq        = FLEXSPI_NOR_FAST_SERIAL_CLK_FREQ,
        .sflashA1Size         = (uint32_t)__flash_size,
        /* Slot meanings differ on the 11xx ROM, so slots are numbered here. */
        .lookupTable = {
            /* 0xEE 0x11 octal DDR read, 32 bit address, 20 dummy cycles counted in DDR half cycles */
            [4 * 0]     = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xee,
                                          CMD_DDR, FLEXSPI_8PAD, 0x11),
            [4 * 0 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
                                          DUMMY_DDR, FLEXSPI_8PAD, 0x28),
            [4 * 0 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04,
                                          STOP, FLEXSPI_1PAD, 0x00),

            /* 0x05 0xFA read status, octal DDR */
            [4 * 2]     = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x05,
                                          CMD_DDR, FLEXSPI_8PAD, 0xfa),
            [4 * 2 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
                                          DUMMY_DDR, FLEXSPI_8PAD, 0x04),
            [4 * 2 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04,
                                          STOP, FLEXSPI_1PAD, 0x00),

            /* 0x06 write enable, still in single line SPI */
            [4 * 3]     = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06,
                                          STOP, FLEXSPI_1PAD, 0x00),

            /* 0x06 0xF9 write enable, octal DDR */
            [4 * 4]     = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x06,
                                          CMD_DDR, FLEXSPI_8PAD, 0xf9),

            /* 0x21 0xDE sector erase, octal DDR */
            [4 * 5]     = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x21,
                                          CMD_DDR, FLEXSPI_8PAD, 0xde),
            [4 * 5 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
                                          STOP, FLEXSPI_1PAD, 0x00),

            /* 0x72 write configuration register 2 (deviceModeSeq) */
            [4 * 6]     = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72,
                                          CMD_SDR, FLEXSPI_1PAD, 0x00),
            [4 * 6 + 1] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00,
                                          CMD_SDR, FLEXSPI_1PAD, 0x00),
            [4 * 6 + 2] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00,
                                          WRITE_SDR, FLEXSPI_1PAD, 0x01),

            /* 0x12 0xED page program, octal DDR */
            [4 * 9]     = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x12,
                                          CMD_DDR, FLEXSPI_8PAD, 0xed),
            [4 * 9 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20,
                                          WRITE_DDR, FLEXSPI_8PAD, 0x04),
        },
    },

    .pageSize           = FLEXSPI_NOR_PAGE_SIZE,
    .sectorSize         = FLEXSPI_NOR_SECTOR_SIZE,
    .ipcmdSerialClkFreq = 1,
    .isUniformBlockSize = false,
    .serialNorType      = 2,
    .blockSize          = 64 * 1024,
    .reserved1          = { 0x07008200 },
};

#else
#error "Unknown FLEXSPI_NOR_PROFILE: see the profile list in xip/flexspi_nor_profiles.h"
#endif
