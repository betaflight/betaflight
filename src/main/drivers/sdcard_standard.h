/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

typedef struct sdcardCSD_t {
    uint8_t data[16];
} sdcardCSD_t;

#define SDCARD_GET_CSD_FIELD(csd, version, fieldname) \
    readBitfield(csd.data, SDCARD_CSD_V ## version ## _ ## fieldname ## _OFFSET, SDCARD_CSD_V ## version ## _ ## fieldname ## _LEN)

// For v1 and Standard Capacity cards
#define SDCARD_CSD_V1_CSD_STRUCTURE_VER_OFFSET           0
#define SDCARD_CSD_V1_CSD_STRUCTURE_VER_LEN              2

#define SDCARD_CSD_V1_TAAC_OFFSET                        8
#define SDCARD_CSD_V1_TAAC_LEN                           8

#define SDCARD_CSD_V1_NSAC_OFFSET                        16
#define SDCARD_CSD_V1_NSAC_LEN                           8

#define SDCARD_CSD_V1_TRAN_SPEED_OFFSET                  24
#define SDCARD_CSD_V1_TRAN_SPEED_LEN                     8

#define SDCARD_CSD_V1_CCC_OFFSET                         32
#define SDCARD_CSD_V1_CCC_LEN                            12

#define SDCARD_CSD_V1_READ_BLOCK_LEN_OFFSET              44
#define SDCARD_CSD_V1_READ_BLOCK_LEN_LEN                 4

#define SDCARD_CSD_V1_READ_BLOCK_PARTIAL_ALLOWED_OFFSET  48
#define SDCARD_CSD_V1_READ_BLOCK_PARTIAL_ALLOWED_LEN     1

#define SDCARD_CSD_V1_WRITE_BLOCK_MISALIGN_OFFSET        49
#define SDCARD_CSD_V1_WRITE_BLOCK_MISALIGN_LEN           1

#define SDCARD_CSD_V1_READ_BLOCK_MISALIGN_OFFSET         50
#define SDCARD_CSD_V1_READ_BLOCK_MISALIGN_LEN            1

#define SDCARD_CSD_V1_DSR_IMPLEMENTED_OFFSET             51
#define SDCARD_CSD_V1_DSR_IMPLEMENTED_LEN                1

#define SDCARD_CSD_V1_CSIZE_OFFSET                       54
#define SDCARD_CSD_V1_CSIZE_LEN                          12

#define SDCARD_CSD_V1_VDD_READ_CURR_MIN_OFFSET           66
#define SDCARD_CSD_V1_VDD_READ_CURR_MIN_LEN              3

#define SDCARD_CSD_V1_VDD_READ_CURR_MAX_OFFSET           69
#define SDCARD_CSD_V1_VDD_READ_CURR_MAX_LEN              3

#define SDCARD_CSD_V1_VDD_WRITE_CURR_MIN_OFFSET          72
#define SDCARD_CSD_V1_VDD_WRITE_CURR_MIN_LEN             3

#define SDCARD_CSD_V1_VDD_WRITE_CURR_MAX_OFFSET          75
#define SDCARD_CSD_V1_VDD_WRITE_CURR_MAX_LEN             3

#define SDCARD_CSD_V1_CSIZE_MULT_OFFSET                  78
#define SDCARD_CSD_V1_CSIZE_MULT_LEN                     3

#define SDCARD_CSD_V1_ERASE_SINGLE_BLOCK_ALLOWED_OFFSET  81
#define SDCARD_CSD_V1_ERASE_SINGLE_BLOCK_ALLOWED_LEN     1

#define SDCARD_CSD_V1_SECTOR_SIZE_OFFSET                 82
#define SDCARD_CSD_V1_SECTOR_SIZE_LEN                    7

#define SDCARD_CSD_V1_WRITE_PROTECT_GROUP_SIZE_OFFSET    89
#define SDCARD_CSD_V1_WRITE_PROTECT_GROUP_SIZE_LEN       7

#define SDCARD_CSD_V1_WRITE_PROTECT_GROUP_ENABLE_OFFSET  96
#define SDCARD_CSD_V1_WRITE_PROTECT_GROUP_ENABLE_LEN     1

#define SDCARD_CSD_V1_R2W_FACTOR_OFFSET                  99
#define SDCARD_CSD_V1_R2W_FACTOR_LEN                     3

#define SDCARD_CSD_V1_WRITE_BLOCK_LEN_OFFSET             102
#define SDCARD_CSD_V1_WRITE_BLOCK_LEN_LEN                4

#define SDCARD_CSD_V1_WRITE_BLOCK_PARTIAL_ALLOWED_OFFSET 106
#define SDCARD_CSD_V1_WRITE_BLOCK_PARTIAL_ALLOWED_LEN    1

#define SDCARD_CSD_V1_FILE_FORMAT_GROUP_OFFSET           112
#define SDCARD_CSD_V1_FILE_FORMAT_GROUP_LEN              1

#define SDCARD_CSD_V1_COPY_OFFSET                        113
#define SDCARD_CSD_V1_COPY_LEN                           1

#define SDCARD_CSD_V1_PERMANENT_WRITE_PROTECT_OFFSET     114
#define SDCARD_CSD_V1_PERMANENT_WRITE_PROTECT_LEN        1

#define SDCARD_CSD_V1_TEMPORARY_WRITE_PROTECT_OFFSET     115
#define SDCARD_CSD_V1_TEMPORARY_WRITE_PROTECT_LEN        1

#define SDCARD_CSD_V1_FILE_FORMAT_OFFSET                 116
#define SDCARD_CSD_V1_FILE_FORMAT_LEN                    2

#define SDCARD_CSD_V1_CRC_OFFSET                         120
#define SDCARD_CSD_V1_CRC_LEN                            7

#define SDCARD_CSD_V1_TRAILER_OFFSET                     127
#define SDCARD_CSD_V1_TRAILER_LEN                        1

// For v2 High Capacity cards
#define SDCARD_CSD_V2_CSD_STRUCTURE_VER_OFFSET           0
#define SDCARD_CSD_V2_CSD_STRUCTURE_VER_LEN              2

#define SDCARD_CSD_V2_TAAC_OFFSET                        8
#define SDCARD_CSD_V2_TAAC_LEN                           8

#define SDCARD_CSD_V2_NSAC_OFFSET                        16
#define SDCARD_CSD_V2_NSAC_LEN                           8

#define SDCARD_CSD_V2_TRAN_SPEED_OFFSET                  24
#define SDCARD_CSD_V2_TRAN_SPEED_LEN                     8

#define SDCARD_CSD_V2_CCC_OFFSET                         32
#define SDCARD_CSD_V2_CCC_LEN                            12

#define SDCARD_CSD_V2_READ_BLOCK_LEN_OFFSET              44
#define SDCARD_CSD_V2_READ_BLOCK_LEN_LEN                 4

#define SDCARD_CSD_V2_READ_BLOCK_PARTIAL_ALLOWED_OFFSET  48
#define SDCARD_CSD_V2_READ_BLOCK_PARTIAL_ALLOWED_LEN     1

#define SDCARD_CSD_V2_WRITE_BLOCK_MISALIGN_OFFSET        49
#define SDCARD_CSD_V2_WRITE_BLOCK_MISALIGN_LEN           1

#define SDCARD_CSD_V2_READ_BLOCK_MISALIGN_OFFSET         50
#define SDCARD_CSD_V2_READ_BLOCK_MISALIGN_LEN            1

#define SDCARD_CSD_V2_DSR_IMPLEMENTED_OFFSET             51
#define SDCARD_CSD_V2_DSR_IMPLEMENTED_LEN                1

#define SDCARD_CSD_V2_CSIZE_OFFSET                       58
#define SDCARD_CSD_V2_CSIZE_LEN                          22

#define SDCARD_CSD_V2_ERASE_SINGLE_BLOCK_ALLOWED_OFFSET  81
#define SDCARD_CSD_V2_ERASE_SINGLE_BLOCK_ALLOWED_LEN     1

#define SDCARD_CSD_V2_SECTOR_SIZE_OFFSET                 82
#define SDCARD_CSD_V2_SECTOR_SIZE_LEN                    7

#define SDCARD_CSD_V2_WRITE_PROTECT_GROUP_SIZE_OFFSET    89
#define SDCARD_CSD_V2_WRITE_PROTECT_GROUP_SIZE_LEN       7

#define SDCARD_CSD_V2_WRITE_PROTECT_GROUP_ENABLE_OFFSET  96
#define SDCARD_CSD_V2_WRITE_PROTECT_GROUP_ENABLE_LEN     1

#define SDCARD_CSD_V2_R2W_FACTOR_OFFSET                  99
#define SDCARD_CSD_V2_R2W_FACTOR_LEN                     3

#define SDCARD_CSD_V2_WRITE_BLOCK_LEN_OFFSET             102
#define SDCARD_CSD_V2_WRITE_BLOCK_LEN_LEN                4

#define SDCARD_CSD_V2_WRITE_BLOCK_PARTIAL_ALLOWED_OFFSET 106
#define SDCARD_CSD_V2_WRITE_BLOCK_PARTIAL_ALLOWED_LEN    1

#define SDCARD_CSD_V2_FILE_FORMAT_GROUP_OFFSET           112
#define SDCARD_CSD_V2_FILE_FORMAT_GROUP_LEN              1

#define SDCARD_CSD_V2_COPY_OFFSET                        113
#define SDCARD_CSD_V2_COPY_LEN                           1

#define SDCARD_CSD_V2_PERMANENT_WRITE_PROTECT_OFFSET     114
#define SDCARD_CSD_V2_PERMANENT_WRITE_PROTECT_LEN        1

#define SDCARD_CSD_V2_TEMPORARY_WRITE_PROTECT_OFFSET     115
#define SDCARD_CSD_V2_TEMPORARY_WRITE_PROTECT_LEN        1

#define SDCARD_CSD_V2_FILE_FORMAT_OFFSET                 116
#define SDCARD_CSD_V2_FILE_FORMAT_LEN                    2

#define SDCARD_CSD_V2_CRC_OFFSET                         120
#define SDCARD_CSD_V2_CRC_LEN                            7

#define SDCARD_CSD_V2_TRAILER_OFFSET                     127
#define SDCARD_CSD_V2_TRAILER_LEN                        1

#define SDCARD_SINGLE_BLOCK_READ_START_TOKEN    0xFE
#define SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN   0xFE
#define SDCARD_MULTIPLE_BLOCK_WRITE_START_TOKEN 0xFC
#define SDCARD_MULTIPLE_BLOCK_WRITE_STOP_TOKEN  0xFD

#define SDCARD_BLOCK_SIZE 512

// Idle bit is set to 1 only when idle during intialization phase:
#define SDCARD_R1_STATUS_BIT_IDLE                 1
#define SDCARD_R1_STATUS_BIT_ERASE_RESET          2
#define SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND      4
#define SDCARD_R1_STATUS_BIT_COM_CRC_ERROR        8
#define SDCARD_R1_STATUS_BIT_ERASE_SEQUENCE_ERROR 16
#define SDCARD_R1_STATUS_BIT_ADDRESS_ERROR        32
#define SDCARD_R1_STATUS_BIT_PARAMETER_ERROR      64

#define SDCARD_CSD_STRUCTURE_VERSION_1      0
#define SDCARD_CSD_STRUCTURE_VERSION_2      1

#define SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6  0x01
#define SDCARD_VOLTAGE_ACCEPTED_LVR         0x02

#define SDCARD_COMMAND_GO_IDLE_STATE             0
#define SDCARD_COMMAND_SEND_OP_COND              1
#define SDCARD_COMMAND_SEND_IF_COND              8
#define SDCARD_COMMAND_SEND_CSD                  9
#define SDCARD_COMMAND_SEND_CID                  10
#define SDCARD_COMMAND_STOP_TRANSMISSION         12
#define SDCARD_COMMAND_SEND_STATUS               13
#define SDCARD_COMMAND_SET_BLOCKLEN              16
#define SDCARD_COMMAND_READ_SINGLE_BLOCK         17
#define SDCARD_COMMAND_READ_MULTIPLE_BLOCK       18
#define SDCARD_COMMAND_WRITE_BLOCK               24
#define SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK      25
#define SDCARD_COMMAND_APP_CMD                   55
#define SDCARD_COMMAND_READ_OCR                  58

#define SDCARD_ACOMMAND_SEND_OP_COND             41
#define SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT 23

// These are worst-case timeouts defined for High Speed cards
#define SDCARD_TIMEOUT_READ_MSEC   100
#define SDCARD_TIMEOUT_WRITE_MSEC  250

uint32_t readBitfield(uint8_t *buffer, unsigned bitIndex, unsigned bitLen);
