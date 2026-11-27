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

// The storage backend interface the mass storage layer in src/main/msc/ uses.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t usbd_msc_block_size_t;

typedef struct {
    int8_t (*Init)(uint8_t lun);
    int8_t (*GetCapacity)(uint8_t lun, uint32_t *blockCount, usbd_msc_block_size_t *blockSize);
    int8_t (*IsReady)(uint8_t lun);
    int8_t (*IsWriteProtected)(uint8_t lun);
    int8_t (*Read)(uint8_t lun, uint8_t *buf, uint32_t blockAddress, uint16_t blockCount);
    int8_t (*Write)(uint8_t lun, uint8_t *buf, uint32_t blockAddress, uint16_t blockCount);
    int8_t (*GetMaxLun)(void);
    int8_t *pInquiry;
} USBD_MSC_StorageType;

// Length of the SCSI INQUIRY response pInquiry points at, from SPC-3.
#define USBD_MSC_INQUIRY_DATA_LEN   36

// Bring the SD card controller up for the storage backend; USDHC needs no DMA channel.
bool mscSdioInitDma(void);
