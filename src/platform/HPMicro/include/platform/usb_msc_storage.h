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

// USB MSC storage backend callback interface (CherryUSB) for HPMicro.

#pragma once

// CherryUSB MSC storage callback type definition for HPMicro
typedef struct usbdStorageCb_s {
    int (*init)(uint8_t lun);
    int (*get_capacity)(uint8_t lun, uint32_t *block_num, uint32_t *block_size);
    int (*read)(uint8_t lun, uint8_t *buf, uint32_t block, uint16_t blen);
    int (*write)(uint8_t lun, const uint8_t *buf, uint32_t block, uint16_t blen);
    int (*get_max_lun)(void);
} USBD_STORAGE_cb_TypeDef;

typedef USBD_STORAGE_cb_TypeDef USBD_MSC_StorageType;
