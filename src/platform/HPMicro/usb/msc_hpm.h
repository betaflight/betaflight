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

// HPMicro USB MSC device init and storage backend selection.

#pragma once

#include <stdint.h>

typedef enum {
    HPM_MSC_STORAGE_SDIO,
    HPM_MSC_STORAGE_SD_SPI,
    HPM_MSC_STORAGE_EMFAT,
} hpmMscStorage_e;

int msc_device_init(uint8_t busid, uint32_t regBase, hpmMscStorage_e storage);
