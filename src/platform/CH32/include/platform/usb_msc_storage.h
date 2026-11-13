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

#pragma once

#include <stdint.h>

#define USBD_STD_INQUIRY_LENGTH     36
#define USBD_MSC_INQUIRY_DATA_LEN   USBD_STD_INQUIRY_LENGTH

typedef uint32_t usbd_msc_block_size_t;

typedef struct USBD_MSC_Storage_s {
    int8_t (*Init)(uint8_t lun);
    int8_t (*GetCapacity)(uint8_t lun, uint32_t *block_num, usbd_msc_block_size_t *block_size);
    int8_t (*IsReady)(uint8_t lun);
    int8_t (*IsWriteProtected)(uint8_t lun);
    int8_t (*Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
    int8_t (*Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
    int8_t (*GetMaxLun)(void);
    int8_t *pInquiry;
} USBD_MSC_StorageType;
