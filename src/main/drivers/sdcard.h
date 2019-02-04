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
#include <stdbool.h>

#include "pg/sdcard.h"

typedef struct sdcardMetadata_s {
    uint32_t numBlocks; /* Card capacity in 512-byte blocks*/
    uint16_t oemID;
    uint8_t manufacturerID;

    char productName[5];

    uint32_t productSerial;
    uint8_t productRevisionMajor;
    uint8_t productRevisionMinor;

    uint16_t productionYear;
    uint8_t productionMonth;
} sdcardMetadata_t;

typedef enum {
    SDCARD_BLOCK_OPERATION_READ,
    SDCARD_BLOCK_OPERATION_WRITE,
    SDCARD_BLOCK_OPERATION_ERASE
} sdcardBlockOperation_e;

typedef enum {
    SDCARD_OPERATION_IN_PROGRESS,
    SDCARD_OPERATION_BUSY,
    SDCARD_OPERATION_SUCCESS,
    SDCARD_OPERATION_FAILURE
} sdcardOperationStatus_e;

typedef void(*sdcard_operationCompleteCallback_c)(sdcardBlockOperation_e operation, uint32_t blockIndex, uint8_t *buffer, uint32_t callbackData);

typedef void(*sdcard_profilerCallback_c)(sdcardBlockOperation_e operation, uint32_t blockIndex, uint32_t duration);

void sdcard_preInit(const sdcardConfig_t *config);
void sdcard_init(const sdcardConfig_t *config);

bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount);
sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);

void sdcardInsertionDetectDeinit(void);
void sdcardInsertionDetectInit(void);

bool sdcard_isInserted(void);
bool sdcard_isInitialized(void);
bool sdcard_isFunctional(void);

bool sdcard_poll(void);
const sdcardMetadata_t* sdcard_getMetadata(void);

void sdcard_setProfilerCallback(sdcard_profilerCallback_c callback);
